/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <quadruped_base/quadruped_base.h>
#include <macros/macros.h>

#include <geometry/geometry.h>

namespace champ
{
    class Odometry
    {
        QuadrupedBase *base_;
        geometry::Transformation prev_foot_position_[4];
        bool prev_foot_contacts_[4];
        float prev_theta_[4];
        unsigned long int prev_time_;
        champ::Velocities prev_vel_;
        float beta_;

        public:
            typedef unsigned long int Time;
            static inline Time now() { return time_us(); }

            Odometry(QuadrupedBase &quadruped_base, Time time = now()):
                base_(&quadruped_base),
                prev_foot_contacts_{1,1,1,1},
                prev_theta_{0,0,0,0},
                prev_time_(time),
                beta_(0.1)
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    prev_foot_position_[i] = base_->legs[i]->foot_from_base();
                }
            }        
            
            bool allFeetInContact()
            {
                if(base_->legs[0]->in_contact() &&
                   base_->legs[1]->in_contact() &&
                   base_->legs[2]->in_contact() &&
                   base_->legs[3]->in_contact())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            bool noFootInContact()
            {
                if(!base_->legs[0]->in_contact() &&
                   !base_->legs[1]->in_contact() &&
                   !base_->legs[2]->in_contact() &&
                   !base_->legs[3]->in_contact())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            void getVelocities(champ::Velocities &vel, Time now = champ::Odometry::now())
            {
                double dt = (now - prev_time_) / 1000000.0;
                if (dt <= 0.0) dt = 0.02;

                // считаем только ноги, которые были в контакте и в прошлом, и сейчас
                unsigned int n = 0;
                float x_sum = 0, y_sum = 0, theta_sum = 0;

                for (unsigned int i = 0; i < 4; i++)
                {
                    geometry::Transformation cur = base_->legs[i]->foot_from_base();
                    bool contact = base_->legs[i]->in_contact();

                    float dx = prev_foot_position_[i].X() - cur.X();
                    float dy = prev_foot_position_[i].Y() - cur.Y();
                    float cur_theta = atan2f(cur.X(), cur.Y());
                    float dth = cur_theta - prev_theta_[i];

                    if (contact && prev_foot_contacts_[i])
                    {
                        n++;
                        x_sum += dx;
                        y_sum += dy;
                        theta_sum += dth;
                    }

                    prev_foot_position_[i] = cur;
                    prev_foot_contacts_[i] = contact;
                    prev_theta_[i] = cur_theta;
                }

                prev_time_ = now;

                if (n == 0)   // нет надёжной опоры: держим прошлое значение
                {
                    vel = prev_vel_;
                    return;
                }

                float sc = base_->gait_config.odom_scaler;
                float vx = (x_sum / n) * sc / dt;
                float vy = (y_sum / n) * sc / dt;
                float wz = (theta_sum / n) / dt;

                vel.linear.x  = (1 - beta_) * vx + beta_ * prev_vel_.linear.x;
                vel.linear.y  = (1 - beta_) * vy + beta_ * prev_vel_.linear.y;
                vel.angular.z = (1 - beta_) * wz + beta_ * prev_vel_.angular.z;
                prev_vel_ = vel;
            }
    };
}

#endif

