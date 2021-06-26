/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _NEWTONPHYSICS_HH_
#define _NEWTONPHYSICS_HH_



#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_newton Newton Physics
    /// \brief Newton Dynamics physics wrapper
    /// \{

    /// \brief Newton physics engine.
    class GZ_PHYSICS_VISIBLE NewtonPhysics : public PhysicsEngine
    {
		/// \enum NewtonParam
		/// \brief Newton physics parameter types.
		public: enum NewtonParam
		{
			 /// \brief Minimum step size
			 MIN_STEP_SIZE

		};

		/// \brief Constructor.
		public: explicit NewtonPhysics(WorldPtr _world);

		/// \brief Destructor.
		public: virtual ~NewtonPhysics();

		// Documentation inherited
		public: virtual void Load(sdf::ElementPtr _sdf);

		// Documentation inherited
		public: virtual void Init() = 0;

		/// \brief Finilize the physics engine.
		public: virtual void Fini();

		// Documentation inherited
		public: virtual void Reset() {}

		// Documentation inherited
		public: virtual void InitForThread() = 0;

		// Documentation inherited
		public: virtual void UpdateCollision() = 0;

		// Documentation inherited
		public: virtual std::string GetType() const = 0;

		// Documentation inherited
		public: virtual void SetSeed(uint32_t _seed) = 0;

		// Documentation inherited
		public: virtual void UpdatePhysics() = 0;

		// Documentation inherited
		public: virtual LinkPtr CreateLink(ModelPtr _parent) = 0;

		// Documentation inherited
		public: virtual CollisionPtr CreateCollision(const std::string &_shapeType, LinkPtr _link) = 0;

		// Documentation inherited
		public: virtual ShapePtr CreateShape(const std::string &_shapeType,	CollisionPtr _collision) = 0;

		// Documentation inherited
		public: virtual JointPtr CreateJoint(const std::string &_type, ModelPtr _parent = ModelPtr()) = 0;

		// Documentation inherited
		public: virtual void SetGravity(const ignition::math::Vector3d &_gravity) = 0;

		// Documentation inherited
		public: virtual bool SetParam(const std::string &_key, const boost::any &_value);

		// Documentation inherited
		public: virtual boost::any GetParam(const std::string &_key) const;

		// Documentation inherited
		public: virtual bool GetParam(const std::string &_key, boost::any &_value) const;

		// Documentation inherited
		public: virtual void DebugPrint() const = 0;

		// Documentation inherited
		protected: virtual void OnRequest(ConstRequestPtr &_msg);

		// Documentation inherited
		protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);
    };
    /// \}
  }
}
#endif
