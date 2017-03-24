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
    void setOutputVal(double output) {m_outputVal = output;}
    double getOutputVal() const {return m_outputVal;}
    void feedForward(const Layer &prevLayer);
    void calcOutputGradient(double targetVal);
    void calcHiddenGradients(const Layer &nextLayer);
    void updateInputWeights(Layer &prevLayer);
    static void setEta(double newEta) {eta = newEta;}
    static void setAlpha(double newAlpha) {alpha = newAlpha;}
    vector <Connection> getWeights() const {return m_outputWeights;}
    void setWeights(const vector <double> &weights);


private:
    double m_outputVal;
    double m_gradient;
    size_t m_myIndex;
    vector<Connection> m_outputWeights;
    static double eta; // [0.0..1.0] overall net training rate
    static double alpha; // momentum

    static double randomWeight() {return rand() / double(RAND_MAX);}
    static double transferFunction(double sum);
    static double transferFunctionDerivative(double x);
    double sumDow(const Layer &nextLayer) const;
};

#endif // NEURON_H
