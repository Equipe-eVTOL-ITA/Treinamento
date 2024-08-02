# Lab do WARAPS - Gazebo + Deep CNN training

## Pré-Requisitos

- Docker Engine
- Docker Compose v1 (`docker-compose`)
- Nvidia Container Toolkit

Siga os passos 1 a 4 para cumprir esses pré-requisitos no seu computador.

**1. Baixar Docker Engine**

```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world
```

**2. Adicionar docker ao user**

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
docker run hello-world
```
**3. Baixar `docker-compose`**

Obs: `docker-compose` é o comando do Docker Compose v1, `docker compose` é da v2. Precisamos baixar a v1 pois o script `bundles.sh` utiliza o comando `docker-compose`.

```bash
sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
docker-compose --version
```

**4. Baixar Nvidia Container Toolkit**

Obs: certifique-se de que tem os drivers da NVidia instalados (cheque com o comando `nvidia-smi`).

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && \
sudo apt-get update && \
sudo apt-get install -y nvidia-container-toolkit && \
sudo nvidia-ctk runtime configure --runtime=docker && \
sudo systemctl restart docker && \
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
# Se instalou ok, deve printar configuracoes da GPU e do driver
```

## Configurando o Workspace WARAPS

**1. Baixar a pasta `waraps_ss2023`**

```bash
mkdir -p ~/waraps_ss2023
wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=11F9jNXKCIciq5UO48QgFuGgf1sBmekvC' -O ~/waraps_ss2023.zip
cd ~ && unzip waraps_ss2023.zip && rm waraps_ss2023.zip
```

**2. Baixar o `bundles2024`**

```bash
cd ~
curl -sfL https://cdn.waraps.org/other/getbundles.sh | sh -
```

**3. Executar o `min_bundles`**

```bash
cd ~/bundles2024
bash bundles.sh up min_bundles
```

**4. Login no Docker e Gitlab**

```bash
docker login gitlab.liu.se:5000 -u wara -p LH1U2K_zXignv2PLF4mc
git@gitlab.liu.se:lrs2/waraps_ss2023.git
```

**5. Configurando o environment**
```bash
cd ~/waraps_ss2023/training
cp env_template .env
```

## Inicializando o Environment

**1. Pegando imagem do docker de `training`**

```bash
cd ~/waraps_ss2023/training
docker-compose pull

# Se o pull deu certo
docker-compose up
```
Se o comando acima deu certo, deve aparecer:
```bash
Recreating training_training_1 ... done
Attaching to training_training_1
```

### Setup único (primeira vez):

Em um novo terminal:

```bash
cd ~/waraps_ss2023/training
docker-compose exec training bash
cd setup_training
./prepare_training
```

## Coletando imagens para treino

### Quer usar joystick?

Before starting, make sure the game-pad environment variables are set up correctly. First, find out the name of the device:

```bash
ls /dev/input/by-id/
```

Find the appropriate device, for example: ```usb-Logitech_Wireless_Gamepad_F710_5A669C5A-joystick```. Put the name of the device to the ```JOY_NAME``` variable but remove -joystick or -event-joystick.

Edit your ~/.bashrc file and add the following but replace the name with yours:
```bash
export JOY_NAME=usb-Logitech_Wireless_Gamepad_F710_5A669C5A
export JOY_DEV=`readlink -f "/dev/input/by-id/${JOY_NAME}-joystick"`
export JOY_EVENT=`readlink -f "/dev/input/by-id/${JOY_NAME}-event-joystick"`
```

After editing the file, open a new terminal window or ```source``` the edited file for the changes to take effect.

After doing one of the above, you can check of the variables are set correctly by doing:
```bash
echo $JOY_NAME
```

The result should be according to the exported variables above.

### Não quer usar joystick?

1. Abra o arquivo `docker-compose` que está em `waraps_ss2023/generate_images/docker-compose.yml`.

2. Na linha 7, remova a parte do comando `joydev:=${JOY_DEV}`

3. Exclua as linhas 10, 11 e 12: 
```yml
    devices:
      - ${JOY_DEV}
      - ${JOY_EVENT}
```

4. Salve o arquivo (Crtl+S)

### Inicializar o container para gerar imagens

```bash
cd ~/waraps_ss2023/generate_images
cp env_template .env
docker-compose up
```
Isso deve abrir uma janela do gazebo com a simulação.