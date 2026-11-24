# Kilder og lokalt arbeidsgrunnlag

Samlet 18. november 2026. Oppsummeringer og prioriteringer i rapporten er prosjektets forslag; artiklenes resultater er ikke validering av vår rigg.

## Filer som ble funnet lokalt

| Arbeidsgrunnlag | Hvordan det er brukt her |
| --- | --- |
| `KALIBRERING_START_HER.md` | Beholdt i [bakgrunn](../bakgrunn/KALIBRERING_START_HER.md); grunnlag for dome, Orin Nano, Light ESS og LightStereo-S |
| `calibmar/` | README og kalibreringsdokumentasjon gjennomgått; referert til originalrepo og oppgitt commit nedenfor |
| `references/Luczynski_OceanEngineering2017.pdf` | [Lokal Pinax-artikkel](Luczynski_OceanEngineering2017.pdf), bevart som bakgrunn; originalkilde nedenfor |

Calibmar-klonen og dens tredjepartsbiblioteker/testbilder er ikke kopiert inn i AUV-repoet. Følg originalkilden for nedlasting. Pinax-PDF-en er bevart i denne mappen. Den bevarte notatfilens relative stier beskriver det opprinnelige lokale arbeidsområdet.

Lokal Pinax-fil: SHA-256 `700a147211c81b4443c616a33aacada28637bda19f1af2fba95e0a4657ee227b`.

## Prioritert leseliste

| Prioritet | Kilde | Hvorfor lese den? |
| --- | --- | --- |
| 1 | [A Calibration Tool for Refractive Underwater Vision](https://arxiv.org/abs/2405.18018), preprint 2024 / ICCV-workshop 2025 | Kamera-, hus- og stereokalibrering med praktisk verktøy |
| 1 | [Refractive Geometry for Underwater Domes](https://arxiv.org/abs/2108.06575), preprint 2021 / journal 2022 | Sentrering, desentrering og geometrien som vår dome-pipeline må håndtere |
| 1 | [LightStereo: Channel Boost Is All You Need for Efficient 2D Cost Aggregation](https://arxiv.org/abs/2406.19833), preprint 2024 / ICRA 2025 | Effektiv lært stereodybde og kandidat for egen tilpasning |
| 1 | [Light ESS / ESS modellkort](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/dnn_stereo_disparity) | Faktisk input/output, konfidens, begrensninger og versjoner |
| 2 | [Integration of a stereo vision system into an autonomous underwater vehicle for pipe manipulation tasks](https://www.sciencedirect.com/science/article/pii/S0045790616302233), 2017 | Nær vår anvendelse: AUV, objekter og manipulasjon |
| 2 | [The Pinax-Model for Accurate and Efficient Refraction Correction of Underwater Cameras in Flat-Pane Housings](https://www.dexrov.eu/wp-content/uploads/2018/10/Luczynski_OceanEngineering2017.pdf), Ocean Engineering 2017 | Effektiv flat-port-korreksjon; gjelder ikke dome direkte. Lokal fil er et forfatterpreprint med 2015-dato |
| 2 | [RAFT-Stereo](https://arxiv.org/abs/2109.07547), 2021; [IGEV-Stereo](https://arxiv.org/abs/2303.06615), 2023 | Sammenligningsmodeller for stereo |
| 3 | [FoundationStereo: Zero-Shot Stereo Matching](https://openaccess.thecvf.com/content/CVPR2025/papers/Wen_FoundationStereo_Zero-Shot_Stereo_Matching_CVPR_2025_paper.pdf), CVPR 2025 | Mulig offline kvalitetsreferanse |
| 3 | [Model-Based Underwater 6D Pose Estimation from RGB](https://arxiv.org/abs/2302.06821), 2023 | Undervannsobjekter, orientering og datasett |
| 3 | [FoundationPose](https://github.com/NVlabs/FoundationPose), CVPR 2024 | Pose og sporing dersom oppgaven trenger mer enn gripepunkt |
| 3 | [SurfSLAM](https://arxiv.org/abs/2601.10814), preprint 2026 | Videre lesning om undervannsstereo og sim-to-real; ikke valgt SLAM-system |

ESS-forskningsbakgrunn: [On the Importance of Stereo for Accurate Depth Estimation](https://arxiv.org/abs/1803.09719), 2018. Dette er bakgrunn, ikke full spesifikasjon av dagens Light ESS.

## Verktøy og reproduksjon

- **Calibmar:** [Originalrepo](https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar), [GitHub-speil](https://github.com/MDSKiel/calibmar), [dokumentasjon](https://mdskiel.github.io/calibmar_documentation/), [releases](https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar/-/releases).
  Lokalt notat oppgir commit `5c5a8200b42b22bac4e1ff62d066c1373a5ce57b`. Verktøyet er ikke bygget eller integrert som del av denne leveransen.
- **Isaac ROS stereo:** [NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth).
- **LightStereo / OpenStereo:** [XiandaGuo/OpenStereo](https://github.com/XiandaGuo/OpenStereo). Velg og registrer eksakt variant og checkpoint.
- **Klassisk stereo:** [NVIDIA VPI](https://docs.nvidia.com/vpi/algo_stereo_disparity.html).
- **Objektsegmentering:** [Ultralytics-dokumentasjon](https://docs.ultralytics.com/tasks/segment).
- **Visuell feedback:** [Uncalibrated Visual Servoing for Underwater Vehicle Manipulator Systems with an Eye in Hand Configuration Camera](https://www.mdpi.com/1424-8220/19/24/5469), 2019.
- **Praktisk marker-/kameraoppsett:** [Automating ROV Manipulation, University of Michigan](https://gidobot.engin.umich.edu/research/automating-rov-manipulation).

Før implementasjon skal teamet låse kompatible programvareversjoner og registrere kodecommit, modellvektversjon, lisens og kalibrerings-ID. Modellvekter og TensorRT-engines er ikke levert eller installert her.
