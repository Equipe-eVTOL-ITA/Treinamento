#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>

class Drone{
public:
    Drone(int id, Eigen::Vector3d posicao){
        this->id_ = id;
        this->posicao_ = posicao;
    }

    int getId(){
        return this->id_;
    }

    Eigen::Vector3d getPosicao(){
        return this->posicao_;
    }

    void setId(int id){
        this->id_ = id;
    }

    void setPosicao(const Eigen::Vector3d& posicao){
        this->posicao_ = posicao;
    }

    virtual void exibirDetalhes(){
        std::cout << "Drone ID: " << this->id_ << std::endl;
        std::cout << "Posicao: [" << this->posicao_[0] << ", " << this->posicao_[1] << ", " << this->posicao_[2] << "]"
        << std::endl;
    }

    ~Drone(){}

private:
    int id_;
    Eigen::Vector3d posicao_;
};


class Frota{
private:
    std::vector<std::shared_ptr<Drone>> drones_;

public:
    void adicionarDrone(std::shared_ptr<Drone> drone){
        this->drones_.push_back(drone);
    }

    void removerDrone(int id){
        for (int i = 0; i < drones_.size(); i++) {
            if(drones_[i]->getId() == id){
                drones_.erase(drones_.begin() + i);
                break;
            }
        }
    }


    void exibirFrota(){
        for (auto& drone : this->drones_) {
            drone->exibirDetalhes();
        }
    }
};

class DroneCarga : public Drone{
private:
    double capacidadeCarga_;
    double cargaAtual_;

public:
    DroneCarga(int id, Eigen::Vector3d posicao, double capacidadeCarga) : Drone(id, posicao){
        this->capacidadeCarga_ = capacidadeCarga;
        this->cargaAtual_ = 0.0;
    }

    double getCapacidadeCarga(){
        return this->capacidadeCarga_;
    }

    double getCargaAtual(){
        return this->cargaAtual_;
    }

    void setCapacidadeCarga(double capacidadeCarga){
        this->capacidadeCarga_ = capacidadeCarga;
    }

    void setCargaAtual(double cargaAtual){
        if (cargaAtual > this->capacidadeCarga_){
            std::cout << "Carga atual maior que a capacidade de carga!" << std::endl;
            return;
        }
        else{
            this->cargaAtual_ = cargaAtual;
        }
    }

    void exibirDetalhes() override{
        Drone::exibirDetalhes();
        std::cout << "Capacidade de Carga: " << this->capacidadeCarga_ << std::endl;
        std::cout << "Carga Atual: " << this->cargaAtual_ << std::endl;
    }
};

int main() {
    DroneCarga drone4(4, Eigen::Vector3d(30.0, 30.0, 20.0), 50.0);
    drone4.setCargaAtual(25.0);
    drone4.exibirDetalhes();

    return 0;
}