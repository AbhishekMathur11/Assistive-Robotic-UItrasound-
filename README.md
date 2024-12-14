# Assistive Robotic UItrasound

This repository provides an implementation of **RUSOpt**, a framework designed to optimize robotic ultrasound probe orientation for both in-plane and out-plane scanning. The methodology leverages **Bayesian Optimization (BO)** to ensure robust probe normalization, improving diagnostic imaging quality.

Check out my full paper here: [!Alt Text]("https://ieeexplore.ieee.org/document/10260479" "RUSOpt: Robotic UltraSound Probe Normalization with Bayesian Optimization for In-Plane and Out-Plane Scanning")

---

## Features
- **Probe Orientation Modeling**: Optimizes ultrasound probe alignment in robotic systems.
- **Bayesian Optimization**: Iterative, probabilistic approach to refine scanning quality.
- **In-Plane and Out-Plane Scanning**: Enhances both imaging modalities.

---

## Methodology

### 1. Problem Formulation
The probe orientation is represented as a function \( f(\mathbf{x}) \), where:
- \( \mathbf{x} \) denotes probe parameters such as angle and position.
- \( f \) represents an imaging quality metric.

#### Objective:
Maximize \( f(\mathbf{x}) \) by finding the optimal orientation:
\[
\mathbf{x}^* = \underset{\mathbf{x} \in \mathcal{X}}{\mathrm{argmax}} \; f(\mathbf{x})
\]

### 2. Bayesian Optimization
Bayesian Optimization iteratively evaluates \( f(\mathbf{x}) \) using a surrogate Gaussian Process (GP) model:
- **Prior**: Encodes initial assumptions about \( f \).
- **Acquisition Function**: Guides exploration, balancing exploitation (known good regions) and exploration (uncertain areas).

#### Acquisition Function:
A common choice is Expected Improvement (EI):
\[
\mathrm{EI}(\mathbf{x}) = \mathbb{E} \left[ \max(0, f(\mathbf{x}) - f(\mathbf{x}^+)) \right]
\]
where \( f(\mathbf{x}^+) \) is the best observed value.

---

### 3. Experimental Setup
- **Robot Platform**: A robotic arm equipped with an ultrasound probe.
- **Metrics**: Imaging quality assessed based on signal clarity and target visibility.
- **Validation**: Evaluated on both synthetic and real-world data.

---

## Results
- Significant improvement in imaging quality for both in-plane and out-plane scenarios.
- Faster convergence compared to traditional grid-based or random search methods.

---
