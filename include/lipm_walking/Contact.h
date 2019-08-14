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

#include <cmath>

#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/Robot.h>

#include <lipm_walking/defs.h>

namespace Eigen
{
  using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;
}

namespace lipm_walking
{
  /** Contact state: set of feet in contact.
   *
   */
  enum class ContactState
  {
    DoubleSupport,
    LeftFoot,
    RightFoot
  };

  /** Contacts wrap foot frames with extra info from the footstep plan.
   *
   */
  struct Contact
  {
    /** Empty constructor.
     *
     * Plücker transform is left uninitialized.
     *
     */
    Contact()
      : refVel({0., 0., 0.}),
        halfLength(0.),
        halfWidth(0.),
        surfaceName(""),
        pose(),
        id(0)
    {
    }

    /** Constructor from Plücker transform.
     *
     * \param pose Plücker transform from inertial to contact frame.
     *
     */
    Contact(const sva::PTransformd & pose)
      : refVel({0., 0., 0.}),
        halfLength(0.),
        halfWidth(0.),
        surfaceName(""),
        pose(pose),
        id(0)
    {
    }

    /** Sagittal unit vector of the contact frame.
     *
     */
    Eigen::Vector3d sagittal() const
    {
      return pose.rotation().row(0);
    }

    /** Lateral unit vector of the contact frame.
     *
     */
    Eigen::Vector3d lateral() const
    {
      return pose.rotation().row(1);
    }

    /** Normal unit vector of the contact frame.
     *
     */
    Eigen::Vector3d normal() const
    {
      return pose.rotation().row(2);
    }

    /** World position of the contact frame.
     *
     */
    const Eigen::Vector3d & position() const
    {
      return pose.translation();
    }

    /** Shorthand for lateral vector.
     *
     */
    Eigen::Vector3d b() const
    {
      return lateral();
    }

    /** Shorthand for normal vector.
     *
     */
    Eigen::Vector3d n() const
    {
      return normal();
    }

    /** Shorthand for sagittal vector.
     *
     */
    Eigen::Vector3d t() const
    {
      return sagittal();
    }

    /** Shorthand for position.
     *
     */
    const Eigen::Vector3d & p() const
    {
      return position();
    }

    /** Position of ankle from foot center frame.
     *
     */
    Eigen::Vector3d anklePos() const
    {
      if (surfaceName == "LeftFootCenter")
      {
        return p() - 0.015 * t() - 0.01 * b();
      }
      else if (surfaceName == "RightFootCenter")
      {
        return p() - 0.015 * t() + 0.01 * b();
      }
      else
      {
        LOG_ERROR("Cannot compute anklePos for surface " << surfaceName);
        return p();
      }
    }

    /** Get frame rooted at the ankle.
     *
     */
    sva::PTransformd anklePose() const
    {
      return {pose.rotation(), anklePos()};
    }

    /** Shorthand for world x-coordinate.
     *
     */
    double x() const
    {
      return position()(0);
    }

    /** Shorthand for world y-coordinate.
     *
     */
    double y() const
    {
      return position()(1);
    }

    /** Shorthand for world z-coordinate.
     *
     */
    double z() const
    {
      return position()(2);
    }

    /** Corner vertex of the contact area.
     *
     */
    Eigen::Vector3d vertex0() const
    {
      return position() + halfLength * t() + halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    Eigen::Vector3d vertex1() const
    {
      return position() + halfLength * t() - halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    Eigen::Vector3d vertex2() const
    {
      return position() - halfLength * t() - halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    Eigen::Vector3d vertex3() const
    {
      return position() - halfLength * t() + halfWidth * b();
    }

    /** Minimum coordinate for vertices of the contact area.
     *
     */
    template <int i>
    double minCoord() const
    {
      return std::min(std::min(vertex0()(i), vertex1()(i)), std::min(vertex2()(i), vertex3()(i)));
    }

    /** Maximum coordinate for vertices of the contact area.
     *
     */
    template <int i>
    double maxCoord() const
    {
      return std::max(std::max(vertex0()(i), vertex1()(i)), std::max(vertex2()(i), vertex3()(i)));
    }

    /** Minimum world x-coordinate of the contact area.
     *
     */
    double xmin() const
    {
      return minCoord<0>();
    }

    /** Maximum world x-coordinate of the contact area.
     *
     */
    double xmax() const
    {
      return maxCoord<0>();
    }

    /** Minimum world y-coordinate of the contact area.
     *
     */
    double ymin() const
    {
      return minCoord<1>();
    }

    /** Maximum world y-coordinate of the contact area.
     *
     */
    double ymax() const
    {
      return maxCoord<1>();
    }

    /** Minimum world z-coordinate of the contact area.
     *
     */
    double zmin() const
    {
      return minCoord<2>();
    }

    /** Maximum world z-coordinate of the contact area.
     *
     */
    double zmax() const
    {
      return maxCoord<2>();
    }

    /** Halfspace representation of contact area in world frame.
     *
     */
    Eigen::HrepXd hrep() const
    {
      Eigen::Matrix<double, 4, 2> localHrepMat, worldHrepMat;
      Eigen::Matrix<double, 4, 1> localHrepVec, worldHrepVec;
      localHrepMat <<
        +1, 0,
        -1, 0,
        0, +1,
        0, -1;
      localHrepVec <<
        halfLength,
        halfLength,
        halfWidth,
        halfWidth;
      if ((normal() - world::e_z).norm() > 1e-3)
      {
        LOG_WARNING("Contact is not horizontal");
      }
      const sva::PTransformd & X_0_c = pose;
      worldHrepMat = localHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
      worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + localHrepVec;
      return Eigen::HrepXd(worldHrepMat, worldHrepVec);
    }

    /** Move contact by a given magnitude in a random direction.
     *
     * \param magnitude Absolute displacement after noising.
     *
     */
    Contact addNoise(double magnitude) const
    {
      Contact noisedContact = *this;
      Eigen::Vector3d unitRandom = Eigen::Vector3d::Random().normalized();
      Eigen::Vector3d displacement = magnitude * unitRandom;
      noisedContact.pose = sva::PTransformd{displacement} * this->pose;
      return noisedContact;
    }

    /** Compute floating base transform that puts the robot in contact.
     *
     * \param robot Robot model (including its multi-body configuration).
     *
     */
    inline sva::PTransformd robotTransform(const mc_rbdyn::Robot & robot) const
    {
      const sva::PTransformd & X_0_c = pose;
      const sva::PTransformd & X_0_fb = robot.posW();
      sva::PTransformd X_s_0 = robot.surfacePose(surfaceName).inv();
      sva::PTransformd X_s_fb = X_0_fb * X_s_0;
      return X_s_fb * X_0_c;
    }

  public:
    Eigen::Vector3d refVel;
    double halfLength;
    double halfWidth;
    mc_rtc::Configuration swingConfig;
    std::string surfaceName;
    sva::PTransformd pose;
    unsigned id;
  };

  /** Apply Plucker transform to contact frame.
   *
   * \param X Transform to apply.
   *
   * \param contact Contact frame.
   *
   */
  inline Contact operator*(const sva::PTransformd & X, const Contact & contact)
  {
    Contact result = contact;
    result.pose = X * contact.pose;
    return result;
  }
}

namespace mc_rtc
{
  template<>
  struct ConfigurationLoader<lipm_walking::Contact>
  {
    static lipm_walking::Contact load(const mc_rtc::Configuration & config)
    {
      lipm_walking::Contact contact;
      contact.pose = config("pose");
      config("half_length", contact.halfLength);
      config("half_width", contact.halfWidth);
      config("ref_vel", contact.refVel);
      config("surface", contact.surfaceName);
      if (config.has("swing"))
      {
        contact.swingConfig = config("swing");
      }
      return contact;
    }

    static mc_rtc::Configuration save(const lipm_walking::Contact & contact)
    {
      mc_rtc::Configuration config;
      config.add("half_length", contact.halfLength);
      config.add("half_width", contact.halfWidth);
      config.add("pose", contact.pose);
      config.add("ref_vel", contact.refVel);
      config.add("surface", contact.surfaceName);
      if (!contact.swingConfig.empty())
      {
        config("swing") = contact.swingConfig;
      }
      return config;
    }
  };
}
