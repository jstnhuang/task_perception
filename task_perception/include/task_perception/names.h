#ifndef _PBI_NAMES_H_
#define _PBI_NAMES_H_

namespace pbi {
namespace bag {
const static char kColorTopic[] = "color_in";
const static char kDepthTopic[] = "depth_in";
const static char kCameraInfoTopic[] = "camera_info";
const static char kCameraTransformTopic[] = "camera_transform";
}

const static char kBaseFrame[] = "base_link";
const static char kCameraFrame[] = "pbi_annotator/camera_frame";
}  // namespace pbi

#endif  // _PBI_NAMES_H_
