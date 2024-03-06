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

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
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
    using gtsam::symbol_shorthand::P;

    mola::SMGeoReferencingOutput ret;

    ASSERT_(!sm.empty());

    // Build list of KF poses with GNNS observations:

    // Convert GNNS obs to ENU:

    // Build and optimize GTSAM graph:
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               initValues;

    // Expression to optimize (i=0...N):
    // P (+) kf_pose{i} = gps_enu{i}

    double gpsUncertaintyMeters = 5.0;
    auto   noise = gtsam::noiseModel::Isotropic::Sigma(3, gpsUncertaintyMeters);

    const gtsam::Vector3_ z0 = gtsam::Vector3(0, 0, 0);

    graph.emplace_shared<FactorGNNS2ENU>(
        P(0), sensorPointOnVeh, observedENU, noise);

    return ret;
}
