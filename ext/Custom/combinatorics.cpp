#include "combinatorics.hpp"


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : print_row
******************************************************************************/

void combinatorics::PascalTriangle::print_row(std::vector<int> dummy){

    for (std::vector<int>::iterator i = dummy.begin(); i != dummy.end(); ++i)
                std::cout<<*i<<" ";

    std::cout<<std::endl;
};


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : PascalTriangle
******************************************************************************/

combinatorics::PascalTriangle::PascalTriangle(int dummy){

    if (dummy > 0){ // if the argument is 0 or negative exit immediately

        std::vector<int> row;
        data.resize(dummy);

        // The first row
        row.resize(1);
        row.at(0) = 1;
        data.at(0) = row;

        // The second row
        if (data.size() > 1){

            row.resize(2);
            row.at(0) = 1; row.at(1) = 1;
            data.at(1) = row;
        }

        // The other rows
        if (data.size() > 2){

            for (int i = 2; i < data.size(); i++){
                row.resize(i + 1); // Theoretically this should work faster than consecutive push_back()s
                row.front() = 1;

                for (int j = 1; j < row.size() - 1; j++){
                    row.at(j) = data.at(i - 1).at(j - 1) + data.at(i - 1).at(j);
                    row.back() = 1;
                    data.at(i) = row;
                }
            }
        }
    }
};


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : ~PascalTriangle
******************************************************************************/

combinatorics::PascalTriangle::~PascalTriangle(){

    for (std::vector<std::vector<int>>::iterator i = data.begin(); i != data.end(); ++i)
        i->clear();

    data.clear();

};


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : print_row
******************************************************************************/

void combinatorics::PascalTriangle::print_row(int dummy){

    if (dummy < data.size()){

        for (std::vector<int>::iterator i = data.at(dummy).begin(); i != data.at(dummy).end(); ++i)
            std::cout<<*i<<" ";

        std::cout<<std::endl;
    };
};


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : print
******************************************************************************/

void combinatorics::PascalTriangle::print(){

    for (int i = 0; i < data.size(); i++){
        print_row(i);
    };
};


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : get_coeff
******************************************************************************/

int combinatorics::PascalTriangle::get_coeff(int dummy1,
                                             int dummy2){

    int result {0};

    if ((dummy1 < data.size()) && (dummy2 < data.at(dummy1).size())){
        result = data.at(dummy1).at(dummy2);
    }

    return result;
};


/*****************************************************************************
* namespace: combinatorics
* class    : PascalTriangle
* method   : get_row
******************************************************************************/

std::vector<int> combinatorics::PascalTriangle::get_row(int dummy){

    std::vector<int> result;

    if (dummy < data.size()){
        result = data.at(dummy);
    };

    return result;
};


int combinatorics::BinomialCoeff::compact_binomial(int n, int k) {

    int result;

    if (k == 0){

        result = 1;

    } else if (k != 0){

        std::vector<int> aSolutions(k);
        aSolutions[0] = n - k + 1;

        for (int i = 1; i < k; ++i) {
            aSolutions[i] = aSolutions[i - 1] * (n - k + 1 + i) / (i + 1);
        };

        result = aSolutions[k - 1];
    };

    return result;
};


/*****************************************************************************
* namespace: combinatorics
* class    : BinomialCoeff
* method   : precompute_pascal_triangle
******************************************************************************/

std::shared_ptr<combinatorics::PascalTriangle> combinatorics::BinomialCoeff::precompute_pascal_triangle(int n){

    std::shared_ptr<combinatorics::PascalTriangle> pascal_triangle = std::make_shared<combinatorics::PascalTriangle>(n);

    return pascal_triangle;

};


/*****************************************************************************
* namespace: combinatorics
* class    : BinomialCoeff
* method   : calculate
******************************************************************************/

int combinatorics::BinomialCoeff::calculate(int n, int k){

    //std::shared_ptr<combinatorics::PascalTriangle> pascal_triangle;

    //pascal_triangle = precompute_pascal_triangle(n);

    //std::shared_ptr<combinatorics::PascalTriangle> pascal_triangle = std::make_shared<combinatorics::PascalTriangle>(n);

    //return pascal_triangle->get_coeff(n, k);
    return compact_binomial(n, k);

};
