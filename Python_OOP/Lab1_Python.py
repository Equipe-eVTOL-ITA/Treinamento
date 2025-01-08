# Parte 1

class Tarefa():
    def __init__(self, titulo, descricao, prioridade):
        self._titulo = titulo
        self._descricao = descricao
        self._prioridade = prioridade

    def __str__(self):
        return f'Tarefa: {self._titulo} | Descrição: {self._descricao} | Prioridade: {self.prioridade}'

    @property
    def titulo(self):
        return self._titulo
    
    @titulo.setter
    def titulo(self, titulo):
        self._titulo = titulo
    
    @property
    def descricao(self):
        return self._descricao
    
    @descricao.setter
    def descricao(self, descricao):
        self._descricao = descricao

    @property
    def prioridade(self):
        return self._prioridade
    
    @prioridade.setter
    def prioridade(self, prioridade):
        if 1 <= prioridade <= 5:
            self._prioridade = prioridade
        else:
            raise ValueError('Prioridade deve ser entre 1 e 5')

    @classmethod
    def contar_tarefas(cls, tarefas, prioridade):
        sum = 0
        for tarefa in tarefas:
            if tarefa.prioridade == prioridade:
                sum += 1    
        return sum

tarefa = Tarefa('Estudar', 'Revisar conceitos de Python', 3)
print(tarefa)

# Parte 2

tarefas = [
    Tarefa("Estudar", "Revisar conceitos de Python", 3),
    Tarefa("Academia", "Treino de musculação", 2),
    Tarefa("Ler", "Terminar livro", 3)
]

print(Tarefa.contar_tarefas(tarefas, 3))

# Parte 3

import numpy as np

prioridades = np.array([tarefa.prioridade for tarefa in tarefas])
copia_prioridades = prioridades.copy()
copia_prioridades[0] = 5
print("Cópia explítica:")
print(prioridades)
print(copia_prioridades)

## Atribuicao normal
copia_prioridades = prioridades
copia_prioridades[0] = 1
print("Cópia implícita:")
print(prioridades)
print(copia_prioridades)
