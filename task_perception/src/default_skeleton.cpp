#include "task_perception/default_skeleton.h"

#include "skin_segmentation_msgs/NerfJointStates.h"

namespace pbi {
// Hard coded joint states for an initial skeleton pose. The usual defaults are
// designed for a camera that is facing forward at person height. However, in
// our setup, the camera is usually looking down 45 degrees with the robot's
// torso all the way up. This lines up with a person standing about 2m  away
// from the robot.
skin_segmentation_msgs::NerfJointStates DefaultSkeleton() {
  // Only the values field is used for updating.
  skin_segmentation_msgs::NerfJointStates js;
  js.values.push_back(-0.08704838901758194);
  js.values.push_back(0.1911003142595291);
  js.values.push_back(1.7519407272338867);
  js.values.push_back(-0.09740731865167618);
  js.values.push_back(-0.08819460868835449);
  js.values.push_back(3.649934768676758);
  js.values.push_back(-0.06651435792446136);
  js.values.push_back(0.012752125039696693);
  js.values.push_back(0.000599895603954792);
  js.values.push_back(-0.014511528424918652);
  js.values.push_back(-0.0437433235347271);
  js.values.push_back(0.012429093942046165);
  js.values.push_back(-0.003522987710312009);
  js.values.push_back(-0.06583462655544281);
  js.values.push_back(-0.011386021971702576);
  js.values.push_back(0.026769915595650673);
  js.values.push_back(1.8283748204339645e-07);
  js.values.push_back(-0.02430078014731407);
  js.values.push_back(0.001422477187588811);
  js.values.push_back(0.0006408896297216415);
  js.values.push_back(0.0016996159683912992);
  js.values.push_back(0.009999999776482582);
  js.values.push_back(-0.011822627857327461);
  js.values.push_back(-0.019999999552965164);
  js.values.push_back(0.094495490193367);
  js.values.push_back(0.1096503734588623);
  js.values.push_back(-0.04435151442885399);
  js.values.push_back(0.14956432580947876);
  js.values.push_back(0.010059650987386703);
  js.values.push_back(-0.38107603788375854);
  js.values.push_back(0.19677674770355225);
  js.values.push_back(-0.019999999552965164);
  js.values.push_back(-0.2731390595436096);
  js.values.push_back(0.15770173072814941);
  js.values.push_back(0.016118481755256653);
  js.values.push_back(0.13312430679798126);
  js.values.push_back(-0.011034148745238781);
  js.values.push_back(-0.004233371000736952);
  js.values.push_back(-0.005884591490030289);
  js.values.push_back(-0.011689063161611557);
  js.values.push_back(0.08068375289440155);
  js.values.push_back(0.026318520307540894);
  js.values.push_back(0.0014631988015025854);
  js.values.push_back(-0.017143961042165756);
  js.values.push_back(0.009059215895831585);
  js.values.push_back(-0.0573158897459507);
  js.values.push_back(-0.0030594400595873594);
  js.values.push_back(0.0490940660238266);
  js.values.push_back(-0.193357914686203);
  js.values.push_back(-0.129545658826828);
  js.values.push_back(-0.009843398816883564);
  js.values.push_back(0.05766604468226433);
  js.values.push_back(-0.2743247449398041);
  js.values.push_back(-0.11433936655521393);
  js.values.push_back(-0.007582078687846661);
  js.values.push_back(0.0017574330559000373);
  js.values.push_back(0.009156696498394012);
  js.values.push_back(0.014868374913930893);
  js.values.push_back(0.032212499529123306);
  js.values.push_back(0.07802033424377441);
  js.values.push_back(0.0788651779294014);
  js.values.push_back(0.151669442653656);
  js.values.push_back(-0.0001744016190059483);
  js.values.push_back(-0.23872897028923035);
  js.values.push_back(0.15257729589939117);
  js.values.push_back(0.019725052639842033);
  js.values.push_back(-0.12070783972740173);
  js.values.push_back(0.021891579031944275);
  js.values.push_back(-0.020828071981668472);
  js.values.push_back(0.05292483791708946);
  js.values.push_back(-0.04305177554488182);
  js.values.push_back(-0.02537298947572708);
  js.values.push_back(-0.01593039184808731);
  js.values.push_back(0.005274764262139797);
  js.values.push_back(-0.041311416774988174);
  js.values.push_back(-0.015498693101108074);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(0.0055440980941057205);
  js.values.push_back(-0.08594159036874771);
  js.values.push_back(-0.023101815953850746);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(0.019922494888305664);
  js.values.push_back(-0.08825071156024933);
  js.values.push_back(-0.004093994852155447);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(0.009174601174890995);
  js.values.push_back(-0.05715002119541168);
  js.values.push_back(-0.00029243185417726636);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(-0.019999999552965164);
  js.values.push_back(0.0009557749144732952);
  js.values.push_back(-0.019999999552965164);
  js.values.push_back(-0.154844731092453);
  js.values.push_back(0.12915706634521484);
  js.values.push_back(0.0097510302439332);
  js.values.push_back(-0.01980549283325672);
  js.values.push_back(0.02677815780043602);
  js.values.push_back(0.0);
  js.values.push_back(0.0);
  js.values.push_back(0.001045851269736886);
  js.values.push_back(0.0);
  js.values.push_back(0.019999999552965164);
  js.values.push_back(0.01946417987346649);
  js.values.push_back(-0.02244577929377556);
  js.values.push_back(-0.08521665632724762);
  js.values.push_back(0.0780545100569725);
  js.values.push_back(0.004965293221175671);
  js.values.push_back(0.021532723680138588);
  js.values.push_back(-0.0012148501118645072);
  js.values.push_back(0.0);
  js.values.push_back(0.008291133679449558);
  js.values.push_back(0.001045851269736886);
  js.values.push_back(0.0);
  return js;
}
}  // namespace pbi
