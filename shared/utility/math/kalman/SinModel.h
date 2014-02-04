#ifndef SINMODEL_H
#define SINMODEL_H

#include <armadillo>

class SinModel {

public:
    // Number of dimensions
    static constexpr size_t size = 2;
    
    SinModel() {} // empty constructor
    
    arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec& measurement);
    
    arma::vec predictedObservation(const arma::vec::fixed<size>& state, const arma::mat& measurementArgs);
    
    arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
    
    void limitState(arma::vec::fixed<size>& state);
    
    unsigned int totalStates();
};

#endif