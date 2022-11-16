/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "include/obstacle_multi_sensor_fusion.h"
#include <iostream>

namespace apollo {
namespace perception {
namespace fusion {

bool ObstacleMultiSensorFusion::Init(
    const ObstacleMultiSensorFusionParam& param) {
  if (fusion_ != nullptr) {
    std::cout << "Already inited";
    return true;
  }
  BaseFusionSystem* fusion = new BaseFusionSystem;
          // BaseFusionSystemRegisterer::GetInstanceByName("ProbabilisticFusion");
  fusion_.reset(fusion);

  FusionInitOptions init_options;
  init_options.main_sensor = "velodyne128";
  if (fusion_ == nullptr || !fusion_->Init(init_options)) {
    std::cout << "Failed to Get Instance or Initialize " << "ProbabilisticFusion";
    return false;
  }
  return true;
}

bool ObstacleMultiSensorFusion::Process(const base::FrameConstPtr& frame,
                                        std::vector<base::ObjectPtr>* objects) {
  FusionOptions options;
  return fusion_->Fuse(options, frame, objects);
}

PERCEPTION_REGISTER_MULTISENSORFUSION(ObstacleMultiSensorFusion);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
