#define NOMINMAX
#include <Windows.h>
#include <iostream>
#include <string>
#include <Kinect.h>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <thread>


// PARTE COMUNICAÇÃO SERIAL
HANDLE abrirPortaSerial(const std::wstring& porta) {
    HANDLE hSerial = CreateFile(porta.c_str(), GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    std::cerr << "[ACERTO] Foi possível abrir porta serial" << std::endl;
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "[ERRO] Não foi possível abrir porta serial" << std::endl;
    }
    return hSerial;
}

void enviarParaSerial(HANDLE hSerial, const std::string& msg) {
    if (hSerial == INVALID_HANDLE_VALUE) return;
    DWORD bytesEnviados = 0;
    BOOL sucesso = WriteFile(hSerial, msg.c_str(), static_cast<DWORD>(msg.size()), &bytesEnviados, NULL);
    if (!sucesso || bytesEnviados != msg.size()) {
        std::cerr << "[ERRO] Falha ao enviar dados pela serial ou envio incompleto.\n";
    }
}
// CALCULO DE ÂNGULOS

float calcularAngulo(const CameraSpacePoint& a, const CameraSpacePoint& b, const CameraSpacePoint& c) {
    float ab[3] = { a.X - b.X, a.Y - b.Y, a.Z - b.Z };
    float cb[3] = { c.X - b.X, c.Y - b.Y, c.Z - b.Z };

    float dot = ab[0] * cb[0] + ab[1] * cb[1] + ab[2] * cb[2];
    float magAB = std::sqrt(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
    float magCB = std::sqrt(cb[0] * cb[0] + cb[1] * cb[1] + cb[2] * cb[2]);

    if (magAB == 0.0f || magCB == 0.0f) return 0.0f;
    float cosTheta = dot / (magAB * magCB);
    if (cosTheta < -1.0f) cosTheta = -1.0f;
    if (cosTheta > 1.0f) cosTheta = 1.0f;
    return acosf(cosTheta) * (180.0f / 3.14159f);
}

// Calcula ângulo entre dois vetores no plano 2D
float calcularAngulo2D(float x1, float y1, float x2, float y2) {
    float dot = x1 * x2 + y1 * y2;
    float mag1 = std::sqrt(x1 * x1 + y1 * y1);
    float mag2 = std::sqrt(x2 * x2 + y2 * y2);
    if (mag1 == 0.0f || mag2 == 0.0f) return 0.0f;
    float cosTheta = dot / (mag1 * mag2);
    if (cosTheta < -1.0f) cosTheta = -1.0f;
    if (cosTheta > 1.0f) cosTheta = 1.0f;
    return acosf(cosTheta) * (180.0f / 3.14159f);
}

// Calcula rotação do antebraço (pronação/supinação)
float calcularRotacaoAntebraco(const CameraSpacePoint& ombro, const CameraSpacePoint& cotovelo, const CameraSpacePoint& pulso, const CameraSpacePoint& mao) {
    float ax = cotovelo.X - ombro.X;
    float ay = cotovelo.Y - ombro.Y;
    float az = cotovelo.Z - ombro.Z;
    float bx = pulso.X - cotovelo.X;
    float by = pulso.Y - cotovelo.Y;
    float bz = pulso.Z - cotovelo.Z;
    float cx = mao.X - pulso.X;
    float cy = mao.Y - pulso.Y;
    float cz = mao.Z - pulso.Z;

    float nx = ay * bz - az * by;
    float ny = az * bx - ax * bz;
    float nz = ax * by - ay * bx;

    float mx = by * cz - bz * cy;
    float my = bz * cx - bx * cz;
    float mz = bx * cy - by * cx;

    float dot = nx * mx + ny * my + nz * mz;
    float magN = std::sqrt(nx * nx + ny * ny + nz * nz);
    float magM = std::sqrt(mx * mx + my * my + mz * mz);
    if (magN == 0.0f || magM == 0.0f) return 0.0f;
    float cosTheta = dot / (magN * magM);
    if (cosTheta < -1.0f) cosTheta = -1.0f;
    if (cosTheta > 1.0f) cosTheta = 1.0f;
    return acosf(cosTheta) * (180.0f / 3.14159f);
}

// === Principal ===
int main() {
    IKinectSensor* sensor = nullptr;
    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        std::cerr << "[ERRO] Kinect não detectado!\n";
        return 1;
    }
    if (FAILED(sensor->Open())) {
        std::cerr << "[ERRO] Falha ao abrir o Kinect!\n";
        return 1;
    }

    IBodyFrameSource* bodySource = nullptr;
    if (FAILED(sensor->get_BodyFrameSource(&bodySource))) {
        std::cerr << "[ERRO] Falha ao obter BodyFrameSource!\n";
        sensor->Close();
        return 1;
    }
    IBodyFrameReader* bodyReader = nullptr;
    if (FAILED(bodySource->OpenReader(&bodyReader))) {
        std::cerr << "[ERRO] Falha ao abrir BodyFrameReader!\n";
        bodySource->Release();
        sensor->Close();
        return 1;
    }

    HANDLE serial = abrirPortaSerial(L"\\\\.\\COM9");
    if (serial == INVALID_HANDLE_VALUE) {
        std::cerr << "[ERRO] Não foi possível abrir a porta serial.\n";
        bodyReader->Release();
        bodySource->Release();
        sensor->Close();
        return 1;
    }

    const int intervaloMs = 33; // Aproximadamente 30 fps

    while (true) {
        auto inicio = std::chrono::high_resolution_clock::now();

        IBodyFrame* frame = nullptr;
        if (SUCCEEDED(bodyReader->AcquireLatestFrame(&frame))) {
            IBody* corpos[6] = { 0 };
            if (SUCCEEDED(frame->GetAndRefreshBodyData(6, corpos))) {
                for (int i = 0; i < 6; i++) {
                    BOOLEAN rastreado = false;
                    if (corpos[i] && SUCCEEDED(corpos[i]->get_IsTracked(&rastreado)) && rastreado) {
                        Joint j[JointType_Count];
                        if (SUCCEEDED(corpos[i]->GetJoints(JointType_Count, j))) {
                            auto ombro = j[JointType_ShoulderRight].Position;
                            auto cotovelo = j[JointType_ElbowRight].Position;
                            auto pulso = j[JointType_WristRight].Position;
                            auto baseColuna = j[JointType_SpineBase].Position;
                            auto mao = j[JointType_HandRight].Position;

                            // 1. Rotação da base (horizontal, plano XZ)
                            float baseXZ1x = ombro.X - baseColuna.X;
                            float baseXZ1z = ombro.Z - baseColuna.Z;
                            float baseXZ2x = cotovelo.X - ombro.X;
                            float baseXZ2z = cotovelo.Z - ombro.Z;
                            float angBase = calcularAngulo2D(baseXZ1x, baseXZ1z, baseXZ2x, baseXZ2z);

                            // 2. Ombro vertical (plano YZ)
                            float ombroYZ1y = ombro.Y - baseColuna.Y;
                            float ombroYZ1z = ombro.Z - baseColuna.Z;
                            float ombroYZ2y = cotovelo.Y - ombro.Y;
                            float ombroYZ2z = cotovelo.Z - ombro.Z;
                            float angOmbro = calcularAngulo2D(ombroYZ1y, ombroYZ1z, ombroYZ2y, ombroYZ2z);

                            // 3. Rotação do bíceps (pronação/supinação)
                            float angBiceps = calcularRotacaoAntebraco(ombro, cotovelo, pulso, mao);

                            // 4. Cotovelo (dobrar/esticar)
                            float angCotovelo = calcularAngulo(ombro, cotovelo, pulso);

                            // 5. Pulso (girar)
                            float angPulso = calcularAngulo(cotovelo, pulso, mao);

                            // Limites e ajustes conforme braço robótico
                            angBase = std::max(0.0f, std::min(angBase, 180.0f));
                            angOmbro = std::max(0.0f, std::min(angOmbro, 180.0f));
                            angBiceps = std::max(0.0f, std::min(angBiceps, 180.0f));
                            angCotovelo = std::max(0.0f, std::min(angCotovelo, 180.0f));
                            angPulso = std::max(0.0f, std::min(angPulso, 180.0f));

                            // Envie para os servos conforme a ordem do braço
                            enviarParaSerial(serial, "articulacao10:" + std::to_string((int)angBase) + "\n");
                            enviarParaSerial(serial, "articulacao9:" + std::to_string((int)angCotovelo) + "\n");
                            enviarParaSerial(serial, "articulacao6:" + std::to_string((int)angBiceps) + "\n");
                            enviarParaSerial(serial, "articulacao11:" + std::to_string((int)angOmbro) + "\n");
                            enviarParaSerial(serial, "articulacao8:" + std::to_string((int)angPulso) + "\n");
                        }
                    }
                    if (corpos[i]) corpos[i]->Release();
                }
            }
            frame->Release();
        }

        auto fim = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duracao = fim - inicio;
        if (duracao.count() < intervaloMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(intervaloMs) - duracao);
        }
    }

    CloseHandle(serial);
    bodyReader->Release();
    bodySource->Release();
    sensor->Close();
    sensor->Release();
    return 0;
}