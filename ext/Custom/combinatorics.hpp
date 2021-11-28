#ifndef COMBINATORICS_H
#define COMBINATORICS_H

#include<iostream>
#include<memory>
#include<vector>

namespace combinatorics{

    /** \brief PascalTriangle implements the Pascal's Triangle
               to solve the binomial coefficient.

     */

    class PascalTriangle{
        private:
            std::vector<std::vector<int>> data;

            void print_row(std::vector<int> dummy);

        public:
            PascalTriangle(int dummy);
            ~PascalTriangle();

            void print_row(int dummy);
            void print();

            int get_coeff(int dummy1,
                          int dummy2);

            std::vector<int> get_row(int dummy);
    };


    /** \brief BinomialCoeff is an abstract class implementing the PascalTriangle
               class to precompute a complete pascal triangle and get the coefficient.

     */

    class BinomialCoeff{
        private:

            /** \brief Calculates the binomial coefficient in
             * the most straightforward way
             *
             *
             */

            int compact_binomial(int n, int k);


            /** \brief Calculates the complete Pascal Triangle
             *
             * \param n are the total number of rows
             *
             */

            std::shared_ptr<combinatorics::PascalTriangle> precompute_pascal_triangle(int n);

        public:
            BinomialCoeff(){};
            ~BinomialCoeff(){};

            /** \brief Interface to calculate the binomial coefficient
             *
             * \param n is the desired row
             * \param k is the desired column
             * \return binomial coefficient
             *
             */
            int calculate(int n, int k);

    };

};

// TO DO

// When number of rows is too big, there are negative numbers


#endif // BINOMIAL_H
