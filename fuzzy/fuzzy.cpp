// Code automatically generated with fuzzylite 6.0.

using namespace fl;

Engine *engine = new Engine;
engine->setName("ObstacleAvoidance");
engine->setDescription("");

InputVariable *obstacle = new InputVariable;
obstacle->setName("obstacle");
obstacle->setDescription("");
obstacle->setEnabled(true);
obstacle->setRange(0.000, 1.000);
obstacle->setLockValueInRange(false);
obstacle->addTerm(new Ramp("left", 1.000, 0.000));
obstacle->addTerm(new Ramp("right", 0.000, 1.000));
engine->addInputVariable(obstacle);

OutputVariable *dir = new OutputVariable;
dir->setName("dir");
dir->setDescription("");
dir->setEnabled(true);
dir->setRange(0.000, 1.000);
dir->setLockValueInRange(false);
dir->setAggregation(new Maximum);
dir->setDefuzzifier(new Centroid(100));
dir->setDefaultValue(fl::nan);
dir->setLockPreviousValue(false);
dir->addTerm(new Ramp("left", 1.000, 0.000));
dir->addTerm(new Ramp("right", 0.000, 1.000));
engine->addOutputVariable(dir);

RuleBlock *mamdani = new RuleBlock;
mamdani->setName("mamdani");
mamdani->setDescription("");
mamdani->setEnabled(true);
mamdani->setConjunction(fl::null);
mamdani->setDisjunction(fl::null);
mamdani->setImplication(new AlgebraicProduct);
mamdani->setActivation(new General);
mamdani->addRule(Rule::parse("if obstacle is left then dir is right", engine));
mamdani->addRule(Rule::parse("if obstacle is right then dir is left", engine));
engine->addRuleBlock(mamdani);
