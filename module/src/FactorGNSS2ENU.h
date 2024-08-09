/*
 * Proprietary Software License
 *
 * Copyright (c) 2018-2024 Jose Luis Blanco-Claraco, University of Almeria.
 * All rights reserved.
 *
 * This file is part of MOLA (Modular Optimization framework
 * for Localization and mApping)
 *
 * Unauthorized copying, distribution, modification, or use of this file,
 * via any medium, is strictly prohibited without the prior written permission
 * of University of Almería (PI: Jose Luis Blanco-Claraco).
 *
 * License:
 * You may use this file in accordance with the terms and conditions
 * set forth in the Proprietary Software License, which is included
 * with this software or can be obtained at University of Almeria.
 *
 * Disclaimer:
 * This software is provided "as is," without warranty of any kind,
 * express or implied, including but not limited to the warranties
 * of merchantability, fitness for a particular purpose, and noninfringement.
 * In no event shall the authors or copyright holders be liable
 * for any claim, damages, or other liability, whether in an action
 * of contract, tort, or otherwise, arising from, out of, or in connection
 * with the software or the use or other dealings in the software.
 */

#pragma once

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

namespace mola
{
class FactorGNSS2ENU : public gtsam::ExpressionFactorN<
                           gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>
{
   private:
    using This = FactorGNSS2ENU;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};

   public:
    /// default constructor
    FactorGNSS2ENU()           = default;
    ~FactorGNSS2ENU() override = default;

    FactorGNSS2ENU(
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
        std::cout << s << "FactorGNSS2ENU(" << keyFormatter(Factor::keys_[0])
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
            "FactorGNSS2ENU", boost::serialization::base_object<Base>(*this));
    }
};
}  // namespace mola