package myProj;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QProvider;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import burlap.shell.visual.VisualExplorer;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;

import java.util.Date;
import java.awt.*;
import java.util.List;

public class BasicBehavior {

	
	
	GridWorldDomain gwdg;
	OOSADomain domain;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	SimulatedEnvironment env;
	int qLearningMaxStepCount;
	String targetFolder;
	public static void main(String[] args) {
		
		/*MazeRunner*/
		BasicBehavior example = new BasicBehavior("It's dangerous to go alone!");
		example.targetFolder = "C:\\MachineLearning\\MDP\\Maze\\Visualizations\\"; //directory to record results
		/**/
		
		/*4 Rooms*/
		//BasicBehavior example = new BasicBehavior();
		//example.targetFolder = "C:\\MachineLearning\\MDP\\4Rooms\\Visualizations\\"; //directory to record results
		/**/
		
		
		Date startTime = new Date();
		
		//we will call planning and learning algorithms here
		//example.valueIterationExample(example.targetFolder);
		//example.policyIterationExample(example.targetFolder);
		//example.QLearningExample(example.targetFolder);
		example.experimenterAndPlotter();
		
		Date endTime = new Date();
		double runTime = endTime.getTime() - startTime.getTime();
		
		System.out.println("Run time: " + runTime + "ms");
		//run the visualizer and plotter 
		example.visualize(example.targetFolder);
		
		/*If you want to run the */
		/*
		Visualizer v = GridWorldVisualizer.getVisualizer(example.gwdg.getMap());
		VisualExplorer exp = new VisualExplorer(example.domain, v, example.initialState);
		
		exp.addKeyAction("w", GridWorldDomain.ACTION_NORTH, "");
		exp.addKeyAction("s", GridWorldDomain.ACTION_SOUTH, "");
		exp.addKeyAction("a", GridWorldDomain.ACTION_WEST, "");
		exp.addKeyAction("d", GridWorldDomain.ACTION_EAST, "");

		exp.initGUI();
		*/
	}
	
	public void visualize(String outputPath){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
		new EpisodeSequenceVisualizer(v, domain, outputPath);
	}
	
	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p){

		List<State> allStates = StateReachability.getReachableStates(initialState, 
				domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
			allStates, 21, 31, valueFunction, p);
		gui.initGUI();

	}
	
	public void manualValueFunctionVis(ValueFunction valueFunction, Policy p){

		List<State> allStates = StateReachability.getReachableStates(
				initialState, domain, hashingFactory);

		//define color function
		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);

		//define a 2D painter of state values, specifying 
		//which variables correspond to the x and y coordinates of the canvas
		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYKeys("agent:x", "agent:y", 
			new VariableDomain(0, gwdg.getWidth()), new VariableDomain(0, gwdg.getHeight()), 
			1, 1);

		//create our ValueFunctionVisualizer that paints for all states
		//using the ValueFunction source and the state value painter we defined
		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, valueFunction);

		//define a policy painter that uses arrow glyphs for each of the grid world actions
		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYKeys("agent:x", "agent:y", 
			new VariableDomain(0, gwdg.getWidth()), new VariableDomain(0, gwdg.getHeight()), 
			1, 1);

		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_NORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_SOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_EAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_WEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);


		//add our policy renderer to it
		gui.setSpp(spp);
		gui.setPolicy(p);

		//set the background color for places where states are not rendered to grey
		gui.setBgColor(Color.GRAY);

		//start it
		gui.initGUI();


	}
	
	public void experimenterAndPlotter(){
		
		//different reward function for more structured performance plots
		((FactoredModel)domain.getModel()).setRf(rf);

		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */
		LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

			public String getAgentName() {
				return "Q-Learning";
			}


			public LearningAgent generateAgent() {
				return new QLearning(domain, 0.99, hashingFactory, 0., 1., qLearningMaxStepCount);
			}
		};
		

		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(
			env, 10, 100, qLearningFactory);
		exp.setUpPlottingConfiguration(500, 250, 2, 1000,
				TrialMode.MOST_RECENT_AND_AVERAGE,
				PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
				PerformanceMetric.AVERAGE_EPISODE_REWARD,
				PerformanceMetric.STEPS_PER_EPISODE
				);

		exp.startExperiment();
		exp.writeStepAndEpisodeDataToCSV(targetFolder + "Q_Learning_Data");
	}

	public void valueIterationExample(String outputPath){
		
		Planner planner = new ValueIteration(domain, 0.99, hashingFactory, 0.001, 100);
		Policy p = planner.planFromState(initialState);

		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "vi");
		
		manualValueFunctionVis((ValueFunction)planner, p);
		
	}
	
	public void policyIterationExample(String outputPath) {
		
		Planner planner = new PolicyIteration(domain, .99, hashingFactory, .001, 100, 100);
		Policy p = planner.planFromState(initialState);
		
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "pi");
		
		manualValueFunctionVis((ValueFunction)planner, p);
		
	}
	
	public void QLearningExample(String outputPath){
		
		LearningAgent agent = new QLearning(domain, 0.99, hashingFactory, 0., 1., qLearningMaxStepCount);

		//run learning for 50 episodes
		for(int i = 0; i < 100; i++){
			Episode e = agent.runLearningEpisode(env);

			e.write(outputPath + "ql_" + i);
			System.out.println("Episode,TimeStep");
			System.out.println(i + "," + e.maxTimeStep());
			
			//reset environment for next learning episode
			env.resetEnvironment();
		}
		
	}
	
	public BasicBehavior(){
		qLearningMaxStepCount = 100;
		
		gwdg = new GridWorldDomain(11, 11);
		gwdg.setMapToFourRooms();
		tf = new GridWorldTerminalFunction(10, 10);
		gwdg.setTf(tf);
		goalCondition = new TFGoalCondition(tf);
		rf = new GoalBasedRF(goalCondition, 10., -.4);
		gwdg.setRf(rf);
		domain = gwdg.generateDomain();
		
		initialState = new GridWorldState(new GridAgent(0, 0), new GridLocation(10, 10, "loc0"));
		hashingFactory = new SimpleHashableStateFactory();

		env = new SimulatedEnvironment(domain, initialState);
		
		VisualActionObserver observer = new VisualActionObserver(domain, 
				GridWorldVisualizer.getVisualizer(gwdg.getMap()));
		observer.initGUI();
		env.addObservers(observer);	
	}
	
	public BasicBehavior(String link){
		qLearningMaxStepCount = 1000;
		
		int[][] map = new int[][] {
			{0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
			{0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1},
			{0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
			{0,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1},
			{0,1,0,0,0,0,0,1,0,0,0,1,0,1,1,1,1,1,1,1,1},
			{0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,1,1,1},
			{0,1,0,0,0,1,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1},
			{0,1,1,1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,1},
			{0,0,0,0,0,1,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1},
			{1,1,1,1,1,1,1,1,0,1,0,1,0,1,1,1,1,1,1,1,1},
			{0,0,0,0,0,0,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1},
			{0,1,1,1,1,1,1,1,0,1,0,1,0,1,1,1,1,1,1,1,1},
			{0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,1,1,1},
			{1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1},
			{0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1,1,1,1},
			{0,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1},
			{0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,1,1,1},
			{0,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1,0,0,0},
			{0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0},
			{0,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0,0},
			{0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,1,1,1},
			{1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1},
			{1,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,1,1,1},
			{1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,1,1,1,1},
			{1,1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,1,1,1,1},
			{1,1,1,1,1,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,1},
			{1,1,0,1,0,0,0,1,0,0,0,1,0,1,0,1,0,1,1,1,1},
			{0,0,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,1},
			{1,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1,0,1,1,1,1},
			{1,1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,1},
			{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
		};
		gwdg = new GridWorldDomain(21, 31);
		gwdg.setMap(map);
		tf = new GridWorldTerminalFunction(18, 19);
		gwdg.setTf(tf);
		goalCondition = new TFGoalCondition(tf);
		rf = new GoalBasedRF(goalCondition, 10., -.1);
		gwdg.setRf(rf);
		domain = gwdg.generateDomain();
		
		initialState = new GridWorldState(new GridAgent(27, 0), new GridLocation(18, 19, "loc0"));
		hashingFactory = new SimpleHashableStateFactory();
		
		env = new SimulatedEnvironment(domain, initialState);
		
		VisualActionObserver observer = new VisualActionObserver(domain, 
				GridWorldVisualizer.getVisualizer(gwdg.getMap()));
		observer.initGUI();
		env.addObservers(observer);	
		
	}
	
}