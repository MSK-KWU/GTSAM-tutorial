#include <iostream>
#include <fstream>
#include <vector>

// GTSAM headers
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
    // ===========================================================================
    // Section 6: Structure from Motion (SfM)
    // ===========================================================================
    cout << "========================================" << endl;
    cout << "Running Structure from Motion (SfM) Example" << endl;
    cout << "========================================" << endl;

    // 1. Define the camera calibration parameters
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    // 2. Define the camera observation noise model
    noiseModel::Isotropic::shared_ptr measurementNoise =
        noiseModel::Isotropic::Sigma(2, 1.0); // 1-pixel error in u,v

    // 3. Create simple ground truth data
    vector<Point3> points;
    points.push_back(Point3(0.0, 0.0, 1.0));
    points.push_back(Point3(-1.0, 0.0, 1.0));
    points.push_back(Point3(0.0, -1.0, 1.0));
    points.push_back(Point3(-1.0, -1.0, 1.0));

    // Create two cameras at different positions
    Pose3 pose1 = Pose3(Rot3(), Point3(-3.0, 0.0, 0.0));
    Pose3 pose2 = Pose3(Rot3(), Point3(3.0, 0.0, 0.0));

    // 4. Create a factor graph
    NonlinearFactorGraph graph;

    // Manually create measurements (avoiding project() to prevent CheiralityException)
    // These are simulated 2D observations
    vector<Point2> measurements1 = {
        Point2(50.0, 50.0),
        Point2(40.0, 50.0),
        Point2(50.0, 40.0),
        Point2(40.0, 40.0)
    };
    
    vector<Point2> measurements2 = {
        Point2(50.0, 50.0),
        Point2(60.0, 50.0),
        Point2(50.0, 40.0),
        Point2(60.0, 40.0)
    };

    // Add projection factors
    for (size_t j = 0; j < points.size(); ++j) {
        graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
            measurements1[j], measurementNoise, Symbol('x', 0), Symbol('p', j), K));
        graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
            measurements2[j], measurementNoise, Symbol('x', 1), Symbol('p', j), K));
    }

    // 5. Add priors to constrain the solution
    noiseModel::Diagonal::shared_ptr poseNoise =
        noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
    graph.add(PriorFactor<Pose3>(Symbol('x', 0), pose1, poseNoise));

    noiseModel::Isotropic::shared_ptr pointNoise =
        noiseModel::Isotropic::Sigma(3, 0.1);
    graph.add(PriorFactor<Point3>(Symbol('p', 0), points[0], pointNoise));

    // 6. Create initial estimates
    Values initial;
    initial.insert(Symbol('x', 0), pose1);
    initial.insert(Symbol('x', 1), pose2);
    for (size_t j = 0; j < points.size(); ++j) {
        initial.insert(Symbol('p', j), points[j]);
    }

    graph.print("\nFactor Graph (SfM):\n");
    initial.print("\nInitial Estimate (SfM):\n");

    // 7. Optimize the graph
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("\nFinal Result (SfM):\n");

    // 8. Save results to file for visualization
    ofstream file("../result/sfm_result.txt");
    
    // Save camera poses
    file << "CAMERAS" << endl;
    for (size_t i = 0; i < 2; ++i) {
        Pose3 pose = result.at<Pose3>(Symbol('x', i));
        Point3 t = pose.translation();
        file << "x" << i << " " << t.x() << " " << t.y() << " " << t.z() << endl;
    }
    
    // Save 3D points
    file << "POINTS" << endl;
    for (size_t j = 0; j < points.size(); ++j) {
        Point3 p = result.at<Point3>(Symbol('p', j));
        file << "p" << j << " " << p.x() << " " << p.y() << " " << p.z() << endl;
    }
    
    file.close();
    cout << "\nResults saved to ../result/sfm_result.txt" << endl;
    cout << "Run 'python3 visualize_sfm.py' to visualize!" << endl;

    return 0;
}
