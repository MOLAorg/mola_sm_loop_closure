/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

// MOLA+MRPT
#include <mola_sm_loop_closure/simplemap_georeference.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

class FactorGNNS2ENU : public gtsam::ExpressionFactorN<
                           gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>
{
   private:
    using This = FactorGNNS2ENU;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};

   public:
    /// default constructor
    FactorGNNS2ENU()           = default;
    ~FactorGNNS2ENU() override = default;

    FactorGNNS2ENU(
        gtsam::Key kPi, const gtsam::Point3& sensorOnVehicle,
        const gtsam::Point3& observedENU, const gtsam::SharedNoiseModel& model)
        : Base({kPi}, model, observedENU), sensorOnVehicle_(sensorOnVehicle)
    {
        this->initialize(expression({kPi}));
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    // Return measurement expression
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Pose3_ Pi(keys[0]);

        return {gtsam::transformFrom(Pi, gtsam::Point3_(sensorOnVehicle_))};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string& s, const gtsam::KeyFormatter& keyFormatter =
                                  gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorGNNS2ENU(" << keyFormatter(Factor::keys_[0])
                  << ")\n";
        gtsam::traits<gtsam::Point3>::Print(
            sensorOnVehicle_, "  sensorOnVehicle: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9)
        const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<gtsam::Point3>::Equals(
                   e->sensorOnVehicle_, sensorOnVehicle_, tol);
    }

   private:
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& BOOST_SERIALIZATION_NVP(measured_);  // params before base class
        ar& BOOST_SERIALIZATION_NVP(sensorOnVehicle_);
        ar& boost::serialization::make_nvp(
            "FactorGNNS2ENU", boost::serialization::base_object<Base>(*this));
    }
};

mola::SMGeoReferencingOutput mola::simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params)
{
    mola::SMGeoReferencingOutput ret;

    ASSERT_(!sm.empty());

    // first gps coord is used as reference:
    std::optional<mrpt::topography::TGeodeticCoords> refCoord;

    struct Frame
    {
        mrpt::poses::CPose3D              pose;
        mrpt::obs::CObservationGPS::Ptr   obs;
        mrpt::obs::gnss::Message_NMEA_GGA gga;
        mrpt::topography::TGeodeticCoords coords;
        mrpt::math::TPoint3D              enu;
        double                            sigma_E = 5.0;
        double                            sigma_N = 5.0;
        double                            sigma_U = 5.0;
    };

    std::vector<Frame> poses;
    poses.reserve(sm.size());

    // Build list of KF poses with GNNS observations:
    for (const auto& [pose, sf, twist] : sm)
    {
        ASSERT_(pose);
        ASSERT_(sf);

        const auto p = pose->getMeanVal();

        mrpt::obs::CObservationGPS::Ptr obs;
        for (size_t i = 0;
             !!(obs = sf->getObservationByClass<mrpt::obs::CObservationGPS>(i));
             i++)
        {
            if (!obs->hasMsgType(mrpt::obs::gnss::NMEA_GGA)) continue;

            auto& f = poses.emplace_back();

            f.pose = p;
            f.obs  = obs;
            f.gga  = obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

            if (obs->covariance_enu)
            {
                f.sigma_E = std::sqrt((*obs->covariance_enu)(0, 0));
                f.sigma_N = std::sqrt((*obs->covariance_enu)(1, 1));
                f.sigma_U = std::sqrt((*obs->covariance_enu)(2, 2));
            }
            else
            {
                f.sigma_E = f.gga.fields.HDOP * 4.5 /*HDOP_REFERENCE_METERS*/;
                f.sigma_N = f.sigma_E;
                f.sigma_U = f.sigma_E;
            }

            ASSERT_(f.sigma_E > 0);
            ASSERT_(f.sigma_N > 0);
            ASSERT_(f.sigma_U > 0);

            f.coords.lat    = f.gga.fields.latitude_degrees;
            f.coords.lon    = f.gga.fields.longitude_degrees;
            f.coords.height = f.gga.fields.altitude_meters;

            // keep first one:
            if (!refCoord.has_value()) refCoord = f.coords;

            // Convert GNNS obs to ENU:
            mrpt::topography::geodeticToENU_WGS84(f.coords, f.enu, *refCoord);

#if 0
            std::cout << "pose: " << p << "\nenu: " << f.enu << "\n";
            // obs->getDescriptionAsText(std::cout);
#endif
        }
    }

    // Build and optimize GTSAM graph:
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               initValues;

    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation

    initValues.insert(T(0), gtsam::Pose3::Identity());

    // Expression to optimize (i=0...N):
    // P (+) kf_pose{i} = gps_enu{i}

    auto noisePoses         = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    auto noiseHorizontality = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1e3, 1e3, 1e3, 1e6, 1e6, params.horizontalitySigmaZ));

    for (size_t i = 0; i < poses.size(); i++)
    {
        const auto& frame = poses.at(i);

        auto noiseOrg = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(frame.sigma_E, frame.sigma_N, frame.sigma_U));

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(frame.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(frame.obs->sensorPose.translation());

        graph.emplace_shared<FactorGNNS2ENU>(
            P(i), sensorPointOnVeh, observedENU, robustNoise);

        const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.pose);

        initValues.insert(P(i), vehiclePose);

        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            T(0), P(i), vehiclePose, noisePoses);

        if (params.addHorizontalityConstraints)
        {
            graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                P(i), gtsam::Pose3::Identity(), noiseHorizontality);
        }
    }

    gtsam::LevenbergMarquardtParams lmParams =
        gtsam::LevenbergMarquardtParams::CeresDefaults();

    gtsam::LevenbergMarquardtOptimizer lm(graph, initValues, lmParams);

    auto optimal = lm.optimize();

    const double errInit = graph.error(initValues);
    const double errEnd  = graph.error(optimal);

    std::cout << "LM iterations: " << lm.iterations() << std::endl;
    std::cout << "Init error   : " << errInit << std::endl;
    std::cout << "Final error  : " << errEnd << std::endl;

    const auto T0 = optimal.at<gtsam::Pose3>(T(0));

    // store results:
    ret.geo_ref.T_enu_to_map =
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T0));

    ret.geo_ref.geo_coord = refCoord.value();

    return ret;
}
