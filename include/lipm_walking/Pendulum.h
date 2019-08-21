/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of lipm_walking_controller.
 *
 * lipm_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * lipm_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lipm_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <mc_tasks/CoMTask.h>

#include <lipm_walking/Contact.h>
#include <lipm_walking/defs.h>

namespace lipm_walking
{
  /** Inverted pendulum model.
   *
   */
  struct Pendulum
  {
    /** Initialize state.
     *
     * \param com Initial CoM position.
     *
     * \param comd Initial CoM velocity.
     *
     * \param comdd Initial CoM acceleration.
     *
     */
    Pendulum(const Eigen::Vector3d & com = Eigen::Vector3d::Zero(), const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(), const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

    /** Complete IPM inputs (ZMP and omega) from CoM and contact plane.
     *
     * \param plane Contact plane.
     *
     */
    void completeIPM(const Contact & plane);

    /** Integrate constant CoM jerk for a given duration.
     *
     * \param comddd CoM jerk.
     *
     * \param dt Integration step.
     *
     * \param floor Frame attached to the contact plane the CoP belongs to.
     *
     */
    void integrateCoMJerk(const Eigen::Vector3d & comddd, double dt);

    /** Integrate in floating-base inverted pendulum mode with constant inputs.
     *
     * \param zmp Zero-tilting Moment Point, i.e. net force application point.
     *
     * \param lambda Normalized stiffness of the pendulum.
     *
     * \param dt Duration of integration step.
     *
     */
    void integrateIPM(Eigen::Vector3d zmp, double lambda, double dt);

    /** Reset to a new state.
     *
     * \param com New CoM position.
     *
     * \param com New CoM velocity.
     *
     * \param comdd Initial CoM acceleration.
     *
     */
    void reset(const Eigen::Vector3d & com, const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(), const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

    /** Reset CoM height above a given contact plane.
     *
     * \param height CoM height above contact plane.
     *
     * \param floor Contact plane.
     *
     */
    void resetCoMHeight(double height, const Contact & contact);

    /** Get CoM position of the inverted pendulum model.
     *
     */
    const Eigen::Vector3d & com() const
    {
      return com_;
    }

    /** Get CoM velocity of the inverted pendulum model.
     *
     */
    const Eigen::Vector3d & comd() const
    {
      return comd_;
    }

    /** Get CoM acceleration of the inverted pendulum.
     *
     */
    const Eigen::Vector3d & comdd() const
    {
      return comdd_;
    }

    /** Instantaneous Divergent Component of Motion.
     *
     */
    Eigen::Vector3d dcm() const
    {
      return com_ + comd_ / omega_;
    }

    /** Natural frequency of last IPM integration.
     *
     */
    double omega() const
    {
      return omega_;
    }

    /** Zero-tilting moment point from last integration.
     *
     * \note In the linear inverted pendulum mode, the ZMP coincides with the
     * centroidal moment pivot (CMP) or its extended version (eCMP).
     *
     */
    const Eigen::Vector3d & zmp() const
    {
      return zmp_;
    }

    /** Velocity of the zero-tilting moment point.
     *
     */
    const Eigen::Vector3d & zmpd() const
    {
      return zmpd_;
    }

  protected:
    Eigen::Vector3d com_; /**< Position of the center of mass */
    Eigen::Vector3d comd_; /**< Velocity of the center of mass */
    Eigen::Vector3d comdd_; /**< Acceleration of the center of mass */
    Eigen::Vector3d comddd_; /**< Jerk of the center of mass */
    Eigen::Vector3d zmp_; /**< Position of the zero-tilting moment point */
    Eigen::Vector3d zmpd_; /**< Velocity of the zero-tilting moment point */
    double omega_; /**< Natural frequency of the linear inverted pendulum */
  };
}
