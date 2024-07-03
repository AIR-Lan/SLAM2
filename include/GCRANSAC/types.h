//
// Created by lzh on 14/07/23.
//



#include "GCRANSAC/estimator/linear_model_estimator.h"
#include "GCRANSAC/solver/solver_linear_model.h"

namespace gcransac
{
    namespace utils
    {typedef estimator::LinearModelEstimator<estimator::solver::LinearModelSolver<3>, // The solver used for fitting a model to a minimal sample
                estimator::solver::LinearModelSolver<3>, // The solver used for fitting a model to a non-minimal sample
                3>										 // The dimensionality of the problem
        Default3DPlaneEstimator;
    }

}


