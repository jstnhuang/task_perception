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
  js.values.push_back(-0.1612870991230011);
  js.values.push_back(0.04867665097117424);
  js.values.push_back(2.3786327838897705);
  js.values.push_back(-0.05447574332356453);
  js.values.push_back(-0.05467025563120842);
  js.values.push_back(3.772512435913086);
  js.values.push_back(-0.029690731316804886);
  js.values.push_back(0.004630787763744593);
  js.values.push_back(-0.005021668039262295);
  js.values.push_back(0.00024390089674852788);
  js.values.push_back(-0.003092646598815918);
  js.values.push_back(0.006138387601822615);
  js.values.push_back(-0.011972501873970032);
  js.values.push_back(-0.11014781892299652);
  js.values.push_back(-0.02609388530254364);
  js.values.push_back(0.05859832465648651);
  js.values.push_back(0.01391539629548788);
  js.values.push_back(-0.04170745611190796);
  js.values.push_back(0.0021036609541624784);
  js.values.push_back(0.0005725773517042398);
  js.values.push_back(0.006801145151257515);
  js.values.push_back(-0.0026443470269441605);
  js.values.push_back(0.022352147847414017);
  js.values.push_back(-0.015505659393966198);
  js.values.push_back(0.025999244302511215);
  js.values.push_back(0.14551089704036713);
  js.values.push_back(-0.2196994572877884);
  js.values.push_back(0.07975294440984726);
  js.values.push_back(-0.019698331132531166);
  js.values.push_back(-0.304276704788208);
  js.values.push_back(0.06515953689813614);
  js.values.push_back(-0.01693911664187908);
  js.values.push_back(-0.22153688967227936);
  js.values.push_back(0.1447325199842453);
  js.values.push_back(0.05018325522542);
  js.values.push_back(0.04819465056061745);
  js.values.push_back(-0.034020453691482544);
  js.values.push_back(0.01699198968708515);
  js.values.push_back(0.007856975309550762);
  js.values.push_back(-0.03827618062496185);
  js.values.push_back(0.12328708916902542);
  js.values.push_back(0.03772320970892906);
  js.values.push_back(0.0014631988015025854);
  js.values.push_back(-0.05484601855278015);
  js.values.push_back(0.014221321791410446);
  js.values.push_back(-0.0809585377573967);
  js.values.push_back(-0.006737143732607365);
  js.values.push_back(0.15774470567703247);
  js.values.push_back(-0.2952468693256378);
  js.values.push_back(-0.18427912890911102);
  js.values.push_back(-0.01636110059916973);
  js.values.push_back(0.18570122122764587);
  js.values.push_back(-0.4172581434249878);
  js.values.push_back(-0.16075001657009125);
  js.values.push_back(-0.013778984546661377);
  js.values.push_back(0.029999999329447746);
  js.values.push_back(-0.0056851026602089405);
  js.values.push_back(0.013106471858918667);
  js.values.push_back(-0.013814818114042282);
  js.values.push_back(0.07387726753950119);
  js.values.push_back(0.002827439457178116);
  js.values.push_back(0.10885165631771088);
  js.values.push_back(0.019999999552965164);
  js.values.push_back(-0.13909637928009033);
  js.values.push_back(0.0708390474319458);
  js.values.push_back(0.019999999552965164);
  js.values.push_back(-0.06438956409692764);
  js.values.push_back(-0.14158183336257935);
  js.values.push_back(0.1297045350074768);
  js.values.push_back(0.04735807329416275);
  js.values.push_back(-0.009104527533054352);
  js.values.push_back(0.005478684324771166);
  js.values.push_back(-0.005285603925585747);
  js.values.push_back(0.011366375721991062);
  js.values.push_back(-0.06579946726560593);
  js.values.push_back(-0.023101815953850746);
  js.values.push_back(-0.0030594400595873594);
  js.values.push_back(0.03586011379957199);
  js.values.push_back(-0.13240891695022583);
  js.values.push_back(-0.03403475135564804);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(0.07711388915777206);
  js.values.push_back(-0.142836794257164);
  js.values.push_back(-0.0040811048820614815);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(0.02646607905626297);
  js.values.push_back(-0.08380138874053955);
  js.values.push_back(-7.616788207087666e-05);
  js.values.push_back(-0.0007981206872500479);
  js.values.push_back(-0.0071240924298763275);
  js.values.push_back(-0.013288114219903946);
  js.values.push_back(-0.00411814684048295);
  js.values.push_back(-0.08095583319664001);
  js.values.push_back(0.02189975045621395);
  js.values.push_back(-0.012012498453259468);
  js.values.push_back(-0.01901327632367611);
  js.values.push_back(0.03627721220254898);
  js.values.push_back(0.004438650328665972);
  js.values.push_back(0.0007578099030070007);
  js.values.push_back(0.001045851269736886);
  js.values.push_back(0.0);
  js.values.push_back(0.005632625427097082);
  js.values.push_back(0.019999999552965164);
  js.values.push_back(-0.018569445237517357);
  js.values.push_back(0.011239204555749893);
  js.values.push_back(0.04150545224547386);
  js.values.push_back(0.002742839977145195);
  js.values.push_back(0.028484275564551353);
  js.values.push_back(0.00017102464335039258);
  js.values.push_back(-0.004937081132084131);
  js.values.push_back(0.012813234701752663);
  js.values.push_back(0.001045851269736886);
  js.values.push_back(0.0);
  return js;
}
}  // namespace pbi
