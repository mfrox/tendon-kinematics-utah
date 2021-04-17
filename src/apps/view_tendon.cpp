#include "cliparser/CliParser.h"
#include "vistendon/TendonBackboneRvizPublisher.h"
#include "vistendon/TendonRvizPublisher.h"

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

namespace E = Eigen;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Publish the shape of a single hard-coded backbone shape to RViz\n"
      "  This is an example of how to do visualize the robot and to\n"
      "  visually inspect this shape.");
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  std::vector<E::Vector3d> backbone_points {
    {0,                      0,                      0                    },
    {0.00013282379244664586, 8.8542686704915893e-05, 0.0048183362822623761},
    {0.00052783960456719817, 0.00017734411430676313, 0.0096222944924363833},
    {0.0011807888469575864,  0.00023774422435236941, 0.014398475543442689 },
    {0.0020848691901629154,  0.00024181221791107988, 0.019134001636465916 },
    {0.0032308218771827562,  0.00016204630127320717, 0.023816186856668101 },
    {0.0046069523943798856, -2.7725061486809984e-05, 0.028432680572448337 },
    {0.0061991284067021016, -0.00035193988107289743, 0.032971585424409165 },
    {0.0079909432484972492, -0.00083330351577287284, 0.037421377328421053 },
    {0.0099636590414980825, -0.0014929670596176901,  0.041770601259497581 },
    {0.012096483349451518,  -0.0023495436147191605,  0.046008424857334432 },
    {0.014366757923367069,  -0.0034187943683490949,  0.050124817023101453 },
    {0.016750135665100578,  -0.0047137365964838585,  0.054110548703798278 },
    {0.019220482205211914,  -0.0062448052947935203,  0.057957099895916533 },
    {0.021750667340813275,  -0.008019190198795885,   0.061657335489431711 },
    {0.024312913976440365,  -0.010040679141174146,   0.065205714795564404 },
    {0.026878975393225701,  -0.012309912655683711,   0.068598291197242908 },
    {0.029420097745194464,  -0.014824600981260724,   0.071832780109992153 },
    {0.031908079887735023,  -0.017579403035378263,   0.074908999842337148 },
    {0.034315681524303088,  -0.020565993965422275,   0.077828976374351017 },
    {0.036616697558473403,  -0.023773401539954657,   0.080596867379787301 },
    {0.038785880570702397,  -0.027188300501592648,   0.083219048076941721 },
    {0.040799881477261858,  -0.030795295294191095,   0.085704083226517097 },
    {0.042637572355448683,  -0.03457712912139755,    0.088062652661978325 },
    {0.044279952290016557,  -0.038514956165630457,   0.090307409998782284 },
    {0.045709898811377929,  -0.042588556794032881,   0.092453001552805364 },
    {0.04691283674803131,   -0.046776697971979746,   0.094515610880525625 },
    {0.047876926155331852,  -0.051057344086618592,   0.096512743273968762 },
    {0.048592856307176031,  -0.055407764210548729,   0.098463071437137284 },
    {0.049053496497500931,  -0.059804466118157332,   0.10038640775831636  },
    {0.04925440285973795,   -0.064223533491367668,   0.1023029919714103   },
    {0.049193918878868759,  -0.068640799325951934,   0.10423320813694903  },
    {0.048872975101987508,  -0.073031822319777631,   0.10619744256943542  },
    {0.048294857894473034,  -0.077371535870307609,   0.10821604897814674  },
    {0.047465604350272333,  -0.081634790293452553,   0.11030850839790331  },
    {0.046394046775197574,  -0.085796573499466469,   0.11249312009473898  },
    {0.045091655369252255,  -0.089831984936857096,   0.11478689615013155  },
    {0.043572546963745788,  -0.093715821745073757,   0.11720552268591571  },
    {0.041853549366646772,  -0.097423574286689893,   0.11976254988524013  },
    {0.039954123043569545,  -0.10093178396060946,    0.12246911529529105  },
    {0.037896267549349043,  -0.10421790770166464,    0.12533401771210487  },
    };
  double radius = 0.01;

  rclcpp::init(arg_count, arg_list);
  auto tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

  vistendon::TendonBackboneRvizPublisher tendon_publisher(tendon_node);
  tendon_publisher.set_color(1.0f, 1.0f, 0.0f, 1.0f); // yellow
  tendon_publisher.set_robot(backbone_points, radius);

  rclcpp::spin(tendon_node);
  rclcpp::shutdown();
  return 0;
}
