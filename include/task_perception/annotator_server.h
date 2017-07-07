#ifndef _PBI_ANNOTATOR_SERVER_H_
#define _PBI_ANNOTATOR_SERVER_H_

#include "task_perception/video_scrubber.h"
#include "task_perception_msgs/AnnotatorEvent.h"

namespace pbi {
class AnnotatorServer {
 public:
  explicit AnnotatorServer(const VideoScrubber& scrubber);
  void Start();
  void HandleEvent(const task_perception_msgs::AnnotatorEvent& event);

 private:
  VideoScrubber scrubber_;
};
}  // namespace pbi

#endif  // _PBI_ANNOTATOR_SERVER_H_
