#include "neuron.h"
#include <cmath>
#include <cassert>
#include <iostream>

double Neuron::eta_ = 0.15;
double Neuron::alpha_ = 0.5;

Neuron::Neuron(size_t numOutputs, size_t myIndex) : myIndex_(myIndex)
{
    for(size_t c = 0; c < numOutputs; ++c)
    {
        outputWeights_.push_back(Connection());
        outputWeights_.back().weight = randomWeight() / 20;
    }
}

//=============================================================================

void Neuron::feedForward(const Layer &prevLayer)
{
    double sum = 0.0;
    for(size_t n = 0; n < prevLayer.size(); ++n)
    {
        sum += prevLayer[n].getOutputVal() * prevLayer[n].outputWeights_[myIndex_].weight;
    }

    outputVal_ = transferFunction(sum);
}

double Neuron::transferFunction(double sum)
{
    return tanh(sum);
}

double Neuron::transferFunctionDerivative(double x)
{
    return 1.0 - x * x;
}

void Neuron::calcOutputGradient(double targetVal)
{
    double delta = targetVal - outputVal_;
    gradient_ = delta * transferFunctionDerivative(outputVal_);
}

void Neuron::calcHiddenGradients(const Layer &nextLayer)
{
    double dow = sumDow(nextLayer);
    gradient_ = dow * transferFunctionDerivative(outputVal_);
}

double Neuron::sumDow(const Layer &nextLayer) const
{
    double sum = 0.0;

    for(size_t n = 0; n < nextLayer.size() - 1; ++n)
    {
        sum += outputWeights_[n].weight * nextLayer[n].gradient_;
    }

    return sum;
}

void Neuron::updateInputWeights(Layer &prevLayer)
{
    for(size_t n = 0; n < prevLayer.size(); ++n)
    {
        Neuron &neuron = prevLayer[n];
        double oldDeltaWeight = neuron.outputWeights_[myIndex_].deltaWeight;

        double newDeltaWeight = eta_ * neuron.outputVal_ * gradient_ +
                alpha_ * oldDeltaWeight;
        neuron.outputWeights_[myIndex_].deltaWeight = newDeltaWeight;
        neuron.outputWeights_[myIndex_].weight += newDeltaWeight;
    }
}

//=============================================================================

void Neuron::setWeights(const vector <double> &weights)
{
    assert(weights.size() == outputWeights_.size());

    for(size_t i = 0; i < outputWeights_.size(); ++i)
    {
        outputWeights_[i].weight = weights[i];
    }
}
