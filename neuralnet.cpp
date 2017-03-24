#include "neuralnet.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <ctime>
#include <clocale>

NeuralNet::NeuralNet(double eta, double alpha) : m_recentAverageError(0),
    m_recentAverageSmoothingFactor(0.5)
{
    Neuron::setEta(eta);
    Neuron::setAlpha(alpha);
}

NeuralNet::NeuralNet(const vector<size_t> &topology, double eta, double alpha) : m_recentAverageError(0),
    m_recentAverageSmoothingFactor(0.5)
{
    srand(time(NULL));
    createNet(topology);
    Neuron::setEta(eta);
    Neuron::setAlpha(alpha);
}

//=============================================================================

void NeuralNet::feedForward(const vector<double> &inputVals)
{
    assert(inputVals.size() == (m_layers[0].size() - 1));

    for(size_t i = 0; i < inputVals.size(); ++i)
    {
        m_layers[0][i].setOutputVal(inputVals[i]);
    }
    for(size_t layerNum = 1; layerNum < m_layers.size(); ++layerNum)
    {
        Layer &prevLayer = m_layers[layerNum - 1];
        for(size_t neuronNum = 0; neuronNum < m_layers[layerNum].size() - 1; ++neuronNum)
        {
            m_layers[layerNum][neuronNum].feedForward(prevLayer);
        }
    }
}

void NeuralNet::backProp(const vector<double> &targetVals)
{
    assert(targetVals.size() == (m_layers.back().size() - 1));

    Layer &outputLayer = m_layers.back();
    m_error = 0.0;

    for(size_t n = 0; n < (outputLayer.size() - 1); ++n)
    {
        double delta = targetVals[n] - outputLayer[n].getOutputVal();
        m_error += delta * delta;
    }
    m_error /= (outputLayer.size() - 1);
    m_error = sqrt(m_error);

    m_recentAverageError =
            (m_recentAverageError * m_recentAverageSmoothingFactor + m_error) /
            (m_recentAverageSmoothingFactor + 1.0);

    // Output Layer Gradients
    for(size_t n = 0; n < outputLayer.size() - 1; ++n)
    {
        outputLayer[n].calcOutputGradient(targetVals[n]);
    }

    // Hidden Layers Gradients
    for(size_t layerNum = m_layers.size() - 2; layerNum > 0; --layerNum)
    {
        Layer &hiddenLayer = m_layers[layerNum];
        Layer &nextLayer = m_layers[layerNum + 1];

        for(size_t n = 0; n < hiddenLayer.size(); ++n)
        {
            hiddenLayer[n].calcHiddenGradients(nextLayer);
        }
    }

    // Update connection weights
    for(size_t layerNum = m_layers.size() - 1; layerNum > 0; --layerNum)
    {
        Layer &layer = m_layers[layerNum];
        Layer &prevLayer = m_layers[layerNum - 1];

        for(size_t n = 0; n < layer.size() - 1; ++n)
        {
            layer[n].updateInputWeights(prevLayer);
        }
    }
}

void NeuralNet::getResults(vector<double> &resultVals) const
{
    resultVals.clear();

    for(size_t n = 0; n < m_layers.back().size() - 1; ++n)
    {
        resultVals.push_back(m_layers.back()[n].getOutputVal());
    }
}

double NeuralNet::train(size_t iterations, double error, const vector< vector<double> > &inputs,
                        const vector< vector<double> > &outputs)
{
    assert(inputs.size() == outputs.size());

    const size_t size = inputs.size();
    double averageError = 0.0;
    for(size_t i = 0; (i < iterations); ++i)
    {
        averageError = 0;
        for(size_t n = 0; n < size; ++n)
        {
            feedForward(inputs[n]);
            backProp(outputs[n]);
            averageError += m_error;
        }
        cout << averageError << " " << averageError / size << endl << flush;
        if((averageError / size) < error)
        {
            break;
        }
    }

    return averageError / size;
}

//=============================================================================

void NeuralNet::createNet(const vector<size_t> &topology)
{
    size_t numberOfLayers = topology.size();
    for(size_t layerNum = 0; layerNum < numberOfLayers; ++layerNum)
    {
        m_layers.push_back(Layer());
        for(size_t neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum)
        {
            size_t numOutputs = (layerNum == (numberOfLayers - 1)) ? 0 : topology[layerNum + 1];
            m_layers.back().push_back(Neuron(numOutputs, neuronNum));
        }

        m_layers.back().back().setOutputVal(1.0);
    }
}

void NeuralNet::saveNet(const char *filename) const
{
    ofstream file;
    file.open(filename, ios::out);
    if(file.is_open())
    {
        for(size_t l = 0; l < m_layers.size(); ++l)
        {
            file << m_layers[l].size() - 1 << ";";
        }
        file << endl;
        for(size_t l = 0; l < m_layers.size() - 1; ++l)
        {
            const Layer &layer = m_layers[l];
            for(size_t n = 0; n < layer.size(); ++n)
            {
                vector <Connection> weights = layer[n].getWeights();
                for(size_t i = 0; i < weights.size(); ++i)
                {
                    file << weights[i].weight << ";";
                }
            }
            file << endl;
        }
        file.close();
    }
    else
    {
        cerr << "Can't open file for save net" << endl << flush;
        exit(-1);
    }
}
void NeuralNet::loadNet(const char *filename)
{
    setlocale(LC_ALL, "C");
    ifstream file;
    file.open(filename, ios::in);
    if(file.is_open())
    {
        if(m_layers.size() != 0)
        {
            m_layers.clear();
        }
        string line;
        getline(file, line);
        vector<size_t> topology;
        {
            istringstream stream(line);
            while(stream)
            {
                string str;
                getline(stream, str, ';');
                topology.push_back(atoi(str.c_str()));
            }
        }
        topology.pop_back();
        createNet(topology);

        for(size_t layerNumber = 0; layerNumber < (m_layers.size() - 1); ++layerNumber)
        {
            Layer &layer = m_layers[layerNumber];
            Layer &nextLayer = m_layers[layerNumber + 1];
            vector <double> layerWeights;
            if(!getline(file, line))
            {
                cerr << "Error while loading weights for layer" << layerNumber <<
                        " of" << m_layers.size() << endl << flush;
                exit(-1);
            }
            istringstream stream(line);
            while(stream)
            {
                string str;
                getline(stream, str, ';');
                layerWeights.push_back(atof(str.c_str()));
            }
            layerWeights.pop_back();
            size_t j = 0;
            for(size_t neuronNumber = 0; neuronNumber < layer.size(); ++neuronNumber)
            {
                vector <double> weights;
                Neuron &neuron = layer[neuronNumber];
                for(size_t i = 0; i < nextLayer.size() - 1; ++i)
                {
                    weights.push_back(layerWeights[j++]);
                }
                neuron.setWeights(weights);
            }
        }
    }
    else
    {
        cerr << "Can't open file for load net" << endl << flush;
        exit(-1);
    }

    file.close();
}
