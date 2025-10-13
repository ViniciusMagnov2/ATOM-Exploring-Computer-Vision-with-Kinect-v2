#ifndef ANGULO_H
#define ANGULO_H

#include <cmath>
#include <Kinect.h>

// Calcula a distância entre dois pontos
inline float distancia(CameraSpacePoint a, CameraSpacePoint b) {
    return sqrtf(
        powf(b.X - a.X, 2) +
        powf(b.Y - a.Y, 2) +
        powf(b.Z - a.Z, 2)
    );
}

// Calcula o ângulo (em graus) entre 3 pontos: A-B-C (com B sendo o vértice)
inline float calcularAngulo(CameraSpacePoint a, CameraSpacePoint b, CameraSpacePoint c) {
    float ab[3] = { a.X - b.X, a.Y - b.Y, a.Z - b.Z };
    float cb[3] = { c.X - b.X, c.Y - b.Y, c.Z - b.Z };

    // Produto escalar
    float dot = ab[0] * cb[0] + ab[1] * cb[1] + ab[2] * cb[2];

    // Módulos
    float magAB = sqrtf(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
    float magCB = sqrtf(cb[0] * cb[0] + cb[1] * cb[1] + cb[2] * cb[2]);

    // Protege contra divisão por zero
    if (magAB == 0 || magCB == 0) return 0;

    float cosTheta = dot / (magAB * magCB);

    // Protege contra valores fora do domínio do arccos
    if (cosTheta < -1.0f) cosTheta = -1.0f;
    if (cosTheta > 1.0f)  cosTheta = 1.0f;

    float rad = acosf(cosTheta); // ângulo em radianos
    return rad * (180.0f / 3.14159f); // converte para graus
}

#endif
