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

// LOOP PRINCIPAL
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

    HANDLE serial = abrirPortaSerial(L"\\\\.\\COMX");
    if (serial == INVALID_HANDLE_VALUE) {
        std::cerr << "[ERRO] Não foi possível abrir a porta serial.\n";
        bodyReader->Release();
        bodySource->Release();
        sensor->Close();
        return 1;
    }

    const int intervaloMs = 33; // 30 FPS +-

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
                            auto baseColuna = j[JointType_SpineBase].Position;

                            // 1. Rotação da base (horizontal, plano XZ)
                            float baseXZ1x = ombro.X - baseColuna.X;
                            float baseXZ1z = ombro.Z - baseColuna.Z;
                            float baseXZ2x = cotovelo.X - ombro.X;
                            float baseXZ2z = cotovelo.Z - ombro.Z;
                            float angBase = calcularAngulo2D(baseXZ1x, baseXZ1z, baseXZ2x, baseXZ2z);

                            float ombroYZ1y = ombro.Y - baseColuna.Y;
                            float ombroYZ1z = ombro.Z - baseColuna.Z;
                            float ombroYZ2y = cotovelo.Y - ombro.Y;
                            float ombroYZ2z = cotovelo.Z - ombro.Z;
                            float angOmbro = calcularAngulo2D(ombroYZ1y, ombroYZ1z, ombroYZ2y, ombroYZ2z);
                            
                            angBase = std::max(0.0f, std::min(angBase, 180.0f));
                            angOmbro = std::max(0.0f, std::min(angOmbro, 180.0f));

                            enviarParaSerial(serial, "articulacao9:" + std::to_string((int)angBase) + "\n");
                            enviarParaSerial(serial, "articulacao10:" + std::to_string((int)angOmbro) + "\n");
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
