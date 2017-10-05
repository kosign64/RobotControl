#ifndef NEURON_H
#define NEURON_H

#include "neuralnet.h"
#include <cstdlib>
#include <vector>

using namespace std;

struct Connection
{
    double weight;
    double deltaWeight;
};

class Neuron;

typedef vector<Neuron> Layer;

class Neuron
{
public:
    Neuron(size_t numOutputs, size_t myIndex);
    void setOutputVal(double output) {outputVal_ = output;}
    double getOutputVal() const {return outputVal_;}
    void feedForward(const Layer &prevLayer);
    void calcOutputGradient(double targetVal);
    void calcHiddenGradients(const Layer &nextLayer);
    void updateInputWeights(Layer &prevLayer);
    static void setEta(double newEta) {eta_ = newEta;}
    static void setAlpha(double newAlpha) {alpha_ = newAlpha;}
    vector <Connection> getWeights() const {return outputWeights_;}
    void setWeights(const vector <double> &weights);


private:
    double outputVal_;
    double gradient_;
    size_t myIndex_;
    vector<Connection> outputWeights_;
    static double eta_; // [0.0..1.0] overall net training rate
    static double alpha_; // momentum

    static double randomWeight() {return rand() / double(RAND_MAX);}
    static double transferFunction(double sum);
    static double transferFunctionDerivative(double x);
    double sumDow(const Layer &nextLayer) const;
};

#endif // NEURON_H
