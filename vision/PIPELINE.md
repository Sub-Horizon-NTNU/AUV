# Visuell vision-pipeline

**Forslag, ikke implementert.** Diagrammet viser første oppsett med ett stereopar. Optisk validering må være godkjent før den vanlige rektifiserte stereogrenen brukes.

![Vision-pipeline fra kalibrering til visuell feedback](assets/vision-pipeline.svg)

## Redigerbart diagram

```mermaid
flowchart TD
    subgraph OFF["Før drift"]
        C["Kalibrer kamera, hus og stereorigg"]
        V{"Uavhengige mål holder toleransen?"}
        K["Lagre validert kalibrering og ID"]
        R["Juster optikk eller bruk refraktiv geometri"]
        C --> V
        V -->|Ja| K
        V -->|Nei| R
        R --> C
    end
    A["Synkroniserte stereobilder + tidsstempel"] --> B["Validert optisk korreksjon og rektifisering"]
    K -.-> B
    B --> D["SGBM/SGM, Light ESS eller LightStereo-S"]
    B --> E["Markør, enkel geometri eller objektsegmentering"]
    D --> F["Gyldige objektpunkter i 3D / definert gripepunkt"]
    E --> F
    F --> G{"Fersk måling og tilstrekkelig kvalitet?"}
    G -->|Ja| H["Sporing + transformasjon til armramme"]
    G -->|Nei| X["Ugyldig status / håndtering av måletap"]
    H --> I["Små armbevegelser med visuell feedback"]
    I -->|Nytt bilde| A
    B -.-> S["Valgfri SLAM / egobevegelse"]
    S -.-> H
```

## Foreslått grensesnitt

Dette er en konseptuell datakontrakt, ikke et eksisterende ROS-meldingsformat.

| Felt | Betydning |
| --- | --- |
| `stamp`, `frame_id` | Eksponeringstid og koordinatsystem for målingen |
| `calibration_id`, `model_id` | Geometri og modellversjon som faktisk ble brukt |
| `object_id`, `class_id` | Sporet objekt og eventuell klasse |
| `position_m` | Definert referansepunkt, oppgitt i meter |
| `orientation` | Bare når orientering er estimert og nødvendig |
| `grasp_point_m` | Gripepunkt som er definert for oppgaven |
| `measurement_status`, `age_ms` | Observert, predikert eller ugyldig; alder på siste observasjon |
| `valid_fraction`, `quality` | Gyldig punktandel og dokumentert kvalitetsmål |
| `covariance` | Empirisk validert usikkerhet hvis tilgjengelig; skal ikke diktes fra nettverksscore |

Kameraenes transformasjoner og armkinematikk må koble kamerarammen til armrammen. Global AUV-pose er valgfri for lokal griping. Ved sporing over tid må bevegelse mellom måletidspunktene håndteres eksplisitt.

Heltrukne piler i SVG viser hovedflyt; stiplede viser kalibrering og valgfri egobevegelse. Kontrollsløyfen bruker en ny observasjon etter bevegelse. Tap av gyldig måling skal ha egen status.
