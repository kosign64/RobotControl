#include "neuron.h"
#include <cmath>
#include <cassert>
#include <iostream>

double Neuron::eta = 0.15;
double Neuron::alpha = 0.5;

Neuron::Neuron(size_t numOutputs, size_t myIndex) : m_myIndex(myIndex)
{
    for(size_t c = 0; c < numOutputs; ++c)
    {
        m_outputWeights.push_back(Connection());
        m_outputWeights.back().weight = randomWeight() / 20;
    }
}

//=============================================================================

void Neuron::feedForward(const Layer &prevLayer)
{
    double sum = 0.0;
    for(size_t n = 0; n < prevLayer.size(); ++n)
    {
        sum += prevLayer[n].getOutputVal() * prevLayer[n].m_outputWeights[m_myIndex].weight;
    }

    m_outputVal = transferFunction(sum);
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
    double delta = targetVal - m_outputVal;
    m_gradient = delta * transferFunctionDerivative(m_outputVal);
}

void Neuron::calcHiddenGradients(const Layer &nextLayer)
{
    double dow = sumDow(nextLayer);
    m_gradient = dow * transferFunctionDerivative(m_outputVal);
}

double Neuron::sumDow(const Layer &nextLayer) const
{
    double sum = 0.0;

    for(size_t n = 0; n < nextLayer.size() - 1; ++n)
    {
        sum += m_outputWeights[n].weight * nextLayer[n].m_gradient;
    }

    return sum;
}

void Neuron::updateInputWeights(Layer &prevLayer)
{
    for(size_t n = 0; n < prevLayer.size(); ++n)
    {
        Neuron &neuron = prevLayer[n];
        double oldDeltaWeight = neuron.m_outputWeights[m_myIndex].deltaWeight;

        double newDeltaWeight = eta * neuron.m_outputVal * m_gradient +
                alpha * oldDeltaWeight;
        neuron.m_outputWeights[m_myIndex].deltaWeight = newDeltaWeight;
        neuron.m_outputWeights[m_myIndex].weight += newDeltaWeight;
    }
}

//=============================================================================

void Neuron::setWeights(const vector <double> &weights)
{
    assert(weights.size() == m_outputWeights.size());

    for(size_t i = 0; i < m_outputWeights.size(); ++i)
    {
        m_outputWeights[i].weight = weights[i];
    }
}
