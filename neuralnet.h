#ifndef NEURALNET_H
#define NEURALNET_H

#include <vector>
#include "neuron.h"
using namespace std;

class Neuron;

typedef vector<Neuron> Layer;

class NeuralNet
{
public:
    NeuralNet(double eta = 0.15, double alpha = 0.5);
    NeuralNet(const vector<size_t> &topology, double eta = 0.15, double alpha = 0.5);
    void feedForward(const vector<double> &inputVals);
    void backProp(const vector<double> &targetVals);
    void getResults(vector<double> &resultVals) const;
    double train(size_t iterations, double error, const vector< vector<double> > &inputs,
                 const vector< vector<double> > &outputs);

    void saveNet(const char *filename) const;
    void loadNet(const char *filename);

private:
    vector<Layer> layers_; // m_layers[layerNum][neuronNum]
    double error_;
    double recentAverageError_;
    double recentAverageSmoothingFactor_;
    void createNet(const vector<size_t> &topology);
};

#endif // NEURALNET_H
