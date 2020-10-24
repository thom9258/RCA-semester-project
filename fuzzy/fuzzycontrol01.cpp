// Code automatically generated with fuzzylite 6.0.

using namespace fl;

Engine *engine = new Engine;
engine->setName("ObstacleAvoidance");
engine->setDescription("");

InputVariable *obstacle_range = new InputVariable;
obstacle_range->setName("obstacle_range");
obstacle_range->setDescription("");
obstacle_range->setEnabled(true);
obstacle_range->setRange(0.000, 10.000);
obstacle_range->setLockValueInRange(false);
obstacle_range->addTerm(new Ramp("far_away", 8.000, 10.000));
obstacle_range->addTerm(new Ramp("in_range", 6.000, 8.000));
obstacle_range->addTerm(new Ramp("close", 3.000, 6.000));
obstacle_range->addTerm(new Ramp("very_close", 0.000, 3.000));
engine->addInputVariable(obstacle_range);

InputVariable *obstacle_angle = new InputVariable;
obstacle_angle->setName("obstacle_angle");
obstacle_angle->setDescription("");
obstacle_angle->setEnabled(true);
obstacle_angle->setRange(-2.500, 2.500);
obstacle_angle->setLockValueInRange(false);
obstacle_angle->addTerm(new Ramp("left", -2.500, 0.000));
obstacle_angle->addTerm(new Ramp("right", 0.000, 2.500));
engine->addInputVariable(obstacle_angle);

OutputVariable *steer = new OutputVariable;
steer->setName("steer");
steer->setDescription("");
steer->setEnabled(true);
steer->setRange(-0.400, 0.400);
steer->setLockValueInRange(false);
steer->setAggregation(new Maximum);
steer->setDefuzzifier(new Centroid(100));
steer->setDefaultValue(fl::nan);
steer->setLockPreviousValue(false);
steer->addTerm(new Ramp("sharp_left", -0.400, -0.350));
steer->addTerm(new Ramp("left", -0.350, -0.200));
steer->addTerm(new Ramp("slight_left", -0.200, 0.000));
steer->addTerm(new Ramp("none", 0.000, 0.000));
steer->addTerm(new Ramp("slight_right", 0.000, 0.200));
steer->addTerm(new Ramp("right", 0.200, 0.350));
steer->addTerm(new Ramp("sharp_right", 0.350, 0.400));
engine->addOutputVariable(steer);

RuleBlock *mamdani = new RuleBlock;
mamdani->setName("mamdani");
mamdani->setDescription("");
mamdani->setEnabled(true);
mamdani->setConjunction(fl::null);
mamdani->setDisjunction(fl::null);
mamdani->setImplication(new AlgebraicProduct);
mamdani->setActivation(new General);
mamdani->addRule(Rule::parse("if obstacle_range is far_away then steer is none",
                             engine));
mamdani->addRule(Rule::parse("if obstacle_range is in_range and obstacle_angle "
                             "is right then steer is slight_left",
                             engine));
mamdani->addRule(Rule::parse("if obstacle_range is in_range and obstacle_angle "
                             "is left then steer is slight_right",
                             engine));
mamdani->addRule(Rule::parse(
    "if obstacle_range is close and obstacle_angle is right then steer is left",
    engine));
mamdani->addRule(Rule::parse(
    "if obstacle_range is close and obstacle_angle is left then steer is right",
    engine));
mamdani->addRule(Rule::parse("if obstacle_range is very_close and "
                             "obstacle_angle is right then steer is sharp_left",
                             engine));
mamdani->addRule(Rule::parse("if obstacle_range is very_close and "
                             "obstacle_angle is left then steer is sharp_right",
                             engine));
engine->addRuleBlock(mamdani);
