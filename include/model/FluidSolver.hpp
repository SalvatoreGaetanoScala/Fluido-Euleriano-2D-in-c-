#pragma once

#include "model/FluidGrid.hpp"

#include <vector>

namespace fluid::model
{

    struct SolverParams
    {
        float diffusion{0.0001f};      // Diffusione del fumo
        float viscosity{0.0001f};      // Viscosità cinematica del fluido
        float buoyancy{0.25f};         // Spinta verso l'alto proporzionale alla densità
        float gravityY{0.0f};          // Gravità verticale (positiva = verso il basso)
        float vorticityStrength{1.5f}; // Conservazione dei vortici (0 = disattivata)
        float densityDissipation{0.003f};
        float velocityDamping{0.001f};
        int solverIterations{16}; // Iterazioni Gauss-Seidel per diffusione/pressione
    };

    class FluidSolver
    {
    public:
        explicit FluidSolver(const GridConfig &gridConfig, const SolverParams &params = {});

        void step(float dt);
        void reset();

        // Interazione sul dominio
        void addDensityBrush(int cx, int cy, int radius, float amount);
        void addVelocityBrush(int cx, int cy, int radius, float forceX, float forceY);
        void setObstacleBrush(int cx, int cy, int radius, bool solid);

        // Sorgente continua opzionale (es. getto/fumo dal basso)
        void setContinuousSource(bool enabled) noexcept { continuousSource_ = enabled; }
        [[nodiscard]] bool hasContinuousSource() const noexcept { return continuousSource_; }

        [[nodiscard]] const FluidGrid &grid() const noexcept { return grid_; }
        [[nodiscard]] FluidGrid &grid() noexcept { return grid_; }

        [[nodiscard]] const SolverParams &params() const noexcept { return params_; }
        void setParams(const SolverParams &params) noexcept { params_ = params; }

        // Calcola la magnitudine del rotore (vorticità) per la visualizzazione
        void computeCurlMagnitude(std::vector<float> &outCurl) const;

    private:
        enum class BoundaryMode
        {
            None,
            FlipX,
            FlipY
        };

        void applyExternalForces(float dt);
        void stepVelocity(float dt);
        void stepDensity(float dt);

        void diffuse(std::vector<float> &field, const std::vector<float> &prevField,
                     float diffRate, float dt, BoundaryMode mode);
        void advect(std::vector<float> &field, const std::vector<float> &prevField,
                    const std::vector<float> &velX, const std::vector<float> &velY,
                    float dt, BoundaryMode mode);
        void project(float dt);
        void applyVorticityConfinement(float dt);
        void enforceBoundary(std::vector<float> &field, BoundaryMode mode);

        GridConfig gridConfig_;
        SolverParams params_;
        FluidGrid grid_;
        bool continuousSource_{true};

        // Buffer temporanei riutilizzati per evitare allocazioni durante la simulazione
        std::vector<float> curl_;
        std::vector<float> omega_;
    };

} // namespace fluid::model
