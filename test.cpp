//
// Created by NikoohematS on 6/6/2016.

// lower_bound/upper_bound example
#include <iostream>     // std::cout
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector

int main () {
    double myints[] = {10.01,20.02,30.03,30.04,20.05,10.06,10.07,20.08};
    std::vector<double> v(myints,myints+8);           // 10 20 30 30 20 10 10 20

    std::sort (v.begin(), v.end());                // 10 10 10 20 20 20 30 30
    for (int i = 0; i<v.size(); i++)
    {
        std::cout << v[i] << '\n';
    }

    std::vector<double>::iterator low,up, low_iter;
    low=std::lower_bound (v.begin(), v.end(), 20.023); //          ^
    up= std::upper_bound (v.begin(), v.end(), 20.09); //                   ^

    std::cout << "lower_bound at position " << (low- v.begin()) << '\n';
    std::cout << "upper_bound at position " << (up - v.begin()) << '\n';
    std::cout << v[low-v.begin()-1] << '\n';
    std::cout << *low << '\n';
    std::cout << *up << '\n';

    return 0;
}