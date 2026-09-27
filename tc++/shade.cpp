#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <tuple>
#include <cstdint> // Necessário para std::uint8_t no SFML 3

// --- PARÂMETROS DA SIMULAÇÃO ---
const int NUM_MAX_PARTICULAS = 50000;
const int TAXA_EMISSAO = 5;
const float CAOS = 0.05f;
const float TEMPO_STEP = 0.1f;

// --- ELASTICIDADE / RIGIDEZ ---
const float RIGIDEZ = 12.0f;
const float AMORTECIMENTO = 4.5f;
const float VEL_MAX = 25.0f;

// Dimensões da Caixa 3D
const float TAM_CAIXA = 100.0f;
const float FUNDO_CAIXA = -100.0f;
const float TOPO_CAIXA = 100.0f;

// Resolução da Janela
const unsigned int WIDTH = 800;
const unsigned int HEIGHT = 600;

// --- ESTRUTURAS AUXILIARES ---
struct Vector3 {
    float x, y, z;
};

struct Projecao2D {
    float px, py;
    float tam;
    float zDepth;
    float velocidade;
};

// Gerador de números aleatórios global
std::mt19937 rng(1337);
float randomFloat(float min, float max) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

// --- CLASSE DA PARTÍCULA 3D ---
class Particula3D {
public:
    float x, y, z;
    float vx, vy, vz;
    float raio;
    bool viva;

    Particula3D() {
        x = randomFloat(-10.0f, 10.0f);
        y = TOPO_CAIXA - 10.0f;
        z = randomFloat(-10.0f, 10.0f);

        vx = randomFloat(-0.5f, 0.5f);
        vy = randomFloat(-1.0f, 0.0f);
        vz = randomFloat(-0.5f, 0.5f);

        raio = 3.5f;
        viva = true;
    }

    void integrarEColidirParedes(Vector3 g) {
        if (!viva) return;

        vx += (g.x + randomFloat(-1.0f, 1.0f) * CAOS) * TEMPO_STEP;
        vy += (g.y + randomFloat(-1.0f, 1.0f) * CAOS) * TEMPO_STEP;
        vz += (g.z + randomFloat(-1.0f, 1.0f) * CAOS) * TEMPO_STEP;

        float velMag = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (velMag > VEL_MAX) {
            float escala = VEL_MAX / velMag;
            vx *= escala;
            vy *= escala;
            vz *= escala;
        }

        x += vx * TEMPO_STEP;
        y += vy * TEMPO_STEP;
        z += vz * TEMPO_STEP;

        const float ATRITO = 0.82f;
        const float RESTITUICAO = -0.2f;

        if (y - raio < FUNDO_CAIXA) { y = FUNDO_CAIXA + raio; vy *= RESTITUICAO; vx *= ATRITO; vz *= ATRITO; }
        else if (y + raio > TOPO_CAIXA) { y = TOPO_CAIXA - raio; vy *= RESTITUICAO; vx *= ATRITO; vz *= ATRITO; }

        if (x - raio < -TAM_CAIXA) { x = -TAM_CAIXA + raio; vx *= RESTITUICAO; vy *= ATRITO; vz *= ATRITO; }
        else if (x + raio > TAM_CAIXA) { x = TAM_CAIXA - raio; vx *= RESTITUICAO; vy *= ATRITO; vz *= ATRITO; }

        if (z - raio < -TAM_CAIXA) { z = -TAM_CAIXA + raio; vz *= RESTITUICAO; vx *= ATRITO; vy *= ATRITO; }
        else if (z + raio > TAM_CAIXA) { z = TAM_CAIXA - raio; vz *= RESTITUICAO; vx *= ATRITO; vy *= ATRITO; }
    }

    Projecao2D projetar(float rotX, float rotY, float distancia) const {
        float radY = rotY * 3.14159265f / 180.0f;
        float radX = rotX * 3.14159265f / 180.0f;

        float x1 = x * std::cos(radY) - z * std::sin(radY);
        float z1 = x * std::sin(radY) + z * std::cos(radY);
        float y1 = y;

        float y2 = y1 * std::cos(radX) - z1 * std::sin(radX);
        float z2 = y1 * std::sin(radX) + z1 * std::cos(radX) + distancia;

        float focal = 350.0f;
        float escala = focal / std::max(1.0f, z2);
        float vel = std::sqrt(vx * vx + vy * vy + vz * vz);

        return {
            WIDTH / 2.0f + x1 * escala,
            HEIGHT / 2.0f - y2 * escala,
            std::max(1.0f, raio * 1.5f * escala),
            z2,
            vel
        };
    }
};

// --- RESOLUÇÃO DE COLISÃO ENTRE DUAS PARTÍCULAS ---
void resolverColisao(Particula3D& p1, Particula3D& p2) {
    if (!p1.viva || !p2.viva) return;

    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    float distSq = dx * dx + dy * dy + dz * dz;
    float distMin = p1.raio + p2.raio;

    if (distSq > 0.0f && distSq < distMin * distMin) {
        float dist = std::sqrt(distSq);
        float nxDir = dx / dist;
        float nyDir = dy / dist;
        float nzDir = dz / dist;
        float compressao = distMin - dist;

        float dvx = p1.vx - p2.vx;
        float dvy = p1.vy - p2.vy;
        float dvz = p1.vz - p2.vz;

        float velRelativa = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
        if (velRelativa < 0.8f && compressao > distMin * 0.6f && p1.raio < 10.0f) {
            float m1 = p1.raio * p1.raio * p1.raio;
            float m2 = p2.raio * p2.raio * p2.raio;
            float mTotal = m1 + m2;
            p1.vx = (p1.vx * m1 + p2.vx * m2) / mTotal;
            p1.vy = (p1.vy * m1 + p2.vy * m2) / mTotal;
            p1.vz = (p1.vz * m1 + p2.vz * m2) / mTotal;
            p1.raio = std::cbrt(mTotal);
            p2.viva = false;
            return;
        }

        float fMola = RIGIDEZ * compressao;
        float velNormal = dvx * nxDir + dvy * nyDir + dvz * nzDir;
        float fTotal = (fMola - AMORTECIMENTO * velNormal) * TEMPO_STEP;

        p1.vx += nxDir * fTotal; p1.vy += nyDir * fTotal; p1.vz += nzDir * fTotal;
        p2.vx -= nxDir * fTotal; p2.vy -= nyDir * fTotal; p2.vz -= nzDir * fTotal;
    }
}

// --- SPATIAL GRID PARA OTIMIZAÇÃO ---
const float TAM_CELULA = 15.0f;

struct HashGrid {
    size_t operator()(const std::tuple<int, int, int>& k) const {
        auto [x, y, z] = k;
        return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    }
};

void atualizarFisicaComGrid(std::vector<Particula3D>& particulas, Vector3 g) {
    std::unordered_map<std::tuple<int, int, int>, std::vector<size_t>, HashGrid> grid;

    for (size_t i = 0; i < particulas.size(); ++i) {
        if (!particulas[i].viva) continue;
        int cx = static_cast<int>(std::floor(particulas[i].x / TAM_CELULA));
        int cy = static_cast<int>(std::floor(particulas[i].y / TAM_CELULA));
        int cz = static_cast<int>(std::floor(particulas[i].z / TAM_CELULA));
        grid[{cx, cy, cz}].push_back(i);
    }

    for (auto& [celula, indices] : grid) {
        auto [cx, cy, cz] = celula;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = grid.find({cx + dx, cy + dy, cz + dz});
                    if (it != grid.end()) {
                        for (size_t i : indices) {
                            for (size_t j : it->second) {
                                if (i < j) {
                                    resolverColisao(particulas[i], particulas[j]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for (auto& p : particulas) {
        p.integrarEColidirParedes(g);
    }
}

// --- RENDERIZAÇÃO DA CAIXA 3D ---
void desenharCaixa3D(sf::RenderWindow& window, float rotX, float rotY, float distancia) {
    std::vector<sf::Vector2f> vertices;
    float radY = rotY * 3.14159265f / 180.0f;
    float radX = rotX * 3.14159265f / 180.0f;

    float coords[2] = {-TAM_CAIXA, TAM_CAIXA};
    for (float x : coords) {
        for (float y : {FUNDO_CAIXA, TOPO_CAIXA}) {
            for (float z : coords) {
                float x1 = x * std::cos(radY) - z * std::sin(radY);
                float z1 = x * std::sin(radY) + z * std::cos(radY);
                float y1 = y;

                float y2 = y1 * std::cos(radX) - z1 * std::sin(radX);
                float z2 = y1 * std::sin(radX) + z1 * std::cos(radX) + distancia;

                float escala = 350.0f / std::max(1.0f, z2);
                vertices.push_back({WIDTH / 2.0f + x1 * escala, HEIGHT / 2.0f - y2 * escala});
            }
        }
    }

    std::pair<int, int> arestas[] = {
        {0,1}, {0,2}, {0,4}, {1,3}, {1,5}, {2,3},
        {2,6}, {3,7}, {4,5}, {4,6}, {5,7}, {6,7}
    };

    sf::Color corLinha(60, 70, 80);
    for (auto& edge : arestas) {
        sf::Vertex line[] = {
            sf::Vertex{vertices[edge.first], corLinha},
            sf::Vertex{vertices[edge.second], corLinha}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
}

// --- HUD / PAINEL ---
void desenharPainelInfo(sf::RenderWindow& window, const sf::Font& font, float fps, size_t numPartic) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "FPS: " << fps << "\n";
    ss << "Partic.: " << numPartic << " / " << NUM_MAX_PARTICULAS << "\n";
    float memKB = (numPartic * sizeof(Particula3D)) / 1024.0f;
    ss << "Memoria: " << memKB << " KB";

    sf::Text text(font, ss.str(), 14);
    text.setFillColor(sf::Color(220, 230, 240));
    text.setPosition({20.0f, 15.0f});

    sf::FloatRect bounds = text.getLocalBounds();
    sf::RectangleShape hudBox(sf::Vector2f{bounds.size.x + 20.0f, bounds.size.y + 20.0f});
    hudBox.setPosition({10.0f, 10.0f});
    hudBox.setFillColor(sf::Color(10, 15, 20, 200));
    hudBox.setOutlineColor(sf::Color(0, 200, 255, 150));
    hudBox.setOutlineThickness(1.0f);

    window.draw(hudBox);
    window.draw(text);
}

// --- MAIN LOOP ---
int main() {
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "Fluido 3D com Shade (SFML 3)");
    window.setFramerateLimit(60);

    sf::Font font;
    if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) {
        std::cout << "Nota: Fonte padrao nao carregada.\n";
    }

    float camRotY = 45.0f;
    float camRotX = 30.0f;
    float camDistancia = 450.0f;

    std::vector<Particula3D> particulas;
    particulas.reserve(NUM_MAX_PARTICULAS);

    bool arrastando = false;
    sf::Vector2i posAnterior;

    sf::Clock clock;
    Vector3 gravidadeMundo = {0.0f, -9.8f, 0.0f};

    while (window.isOpen()) {
        float fps = 1.0f / clock.restart().asSeconds();

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            else if (const auto* mousePressed = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mousePressed->button == sf::Mouse::Button::Left) {
                    arrastando = true;
                    posAnterior = sf::Mouse::getPosition(window);
                }
            }
            else if (const auto* mouseReleased = event->getIf<sf::Event::MouseButtonReleased>()) {
                if (mouseReleased->button == sf::Mouse::Button::Left) {
                    arrastando = false;
                }
            }
            else if (event->is<sf::Event::MouseMoved>()) {
                if (arrastando) {
                    sf::Vector2i posAtual = sf::Mouse::getPosition(window);
                    float dx = static_cast<float>(posAtual.x - posAnterior.x);
                    float dy = static_cast<float>(posAtual.y - posAnterior.y);

                    camRotY += dx * 0.5f;
                    camRotX = std::clamp(camRotX + dy * 0.5f, -85.0f, 85.0f);
                    posAnterior = posAtual;
                }
            }
            else if (const auto* scroll = event->getIf<sf::Event::MouseWheelScrolled>()) {
                camDistancia = std::clamp(camDistancia - scroll->delta * 20.0f, 200.0f, 800.0f);
            }
        }

        if (particulas.size() < NUM_MAX_PARTICULAS) {
            for (int i = 0; i < TAXA_EMISSAO; ++i) {
                particulas.emplace_back();
            }
        }

        atualizarFisicaComGrid(particulas, gravidadeMundo);

        particulas.erase(
            std::remove_if(particulas.begin(), particulas.end(), [](const Particula3D& p) { return !p.viva; }),
            particulas.end()
        );

        struct RenderElem { Projecao2D proj; };
        std::vector<RenderElem> listaRender;
        listaRender.reserve(particulas.size());

        for (const auto& p : particulas) {
            listaRender.push_back({p.projetar(camRotX, camRotY, camDistancia)});
        }

        std::sort(listaRender.begin(), listaRender.end(), [](const RenderElem& a, const RenderElem& b) {
            return a.proj.zDepth > b.proj.zDepth;
        });

        window.clear(sf::Color(12, 14, 18));

        desenharCaixa3D(window, camRotX, camRotY, camDistancia);

        sf::VertexArray vaQuads(sf::PrimitiveType::Triangles, listaRender.size() * 6);
        size_t vIdx = 0;

        for (const auto& elem : listaRender) {
            float px = elem.proj.px;
            float py = elem.proj.py;
            float r = elem.proj.tam;

            // --- CÁLCULO DE SHADE 3D ---
            float minZ = camDistancia - TAM_CAIXA * 1.5f;
            float maxZ = camDistancia + TAM_CAIXA * 1.5f;
            float fatorZ = 1.0f - std::clamp((elem.proj.zDepth - minZ) / (maxZ - minZ), 0.0f, 0.75f);
            float fatorVel = std::min(1.3f, 1.0f + elem.proj.velocidade * 0.015f);

            // Uso de std::uint8_t compatível com SFML 3
            auto red   = static_cast<std::uint8_t>(std::clamp(40.0f  * fatorZ * fatorVel, 0.0f, 255.0f));
            auto green = static_cast<std::uint8_t>(std::clamp(180.0f * fatorZ * fatorVel, 0.0f, 255.0f));
            auto blue  = static_cast<std::uint8_t>(std::clamp(255.0f * fatorZ * fatorVel, 0.0f, 255.0f));

            sf::Color corShaded(red, green, blue);

            vaQuads[vIdx++] = sf::Vertex{sf::Vector2f(px - r, py - r), corShaded};
            vaQuads[vIdx++] = sf::Vertex{sf::Vector2f(px + r, py - r), corShaded};
            vaQuads[vIdx++] = sf::Vertex{sf::Vector2f(px - r, py + r), corShaded};

            vaQuads[vIdx++] = sf::Vertex{sf::Vector2f(px + r, py - r), corShaded};
            vaQuads[vIdx++] = sf::Vertex{sf::Vector2f(px + r, py + r), corShaded};
            vaQuads[vIdx++] = sf::Vertex{sf::Vector2f(px - r, py + r), corShaded};
        }

        window.draw(vaQuads);

        desenharPainelInfo(window, font, fps, particulas.size());

        window.display();
    }

    return 0;
}