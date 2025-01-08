#include <iostream>
int main(){
    int x = 10;
    int& ref = x;  // ref é uma referência para x
    ref = 20;  // Modifica x
    std::cout << "Valor de x: " << x << std::endl;
}