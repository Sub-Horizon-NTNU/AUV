# Kandidater og modellvalg

Status 18. november 2026: Ingen kandidat er testet på vår undervannsrigg. Prioriteringen er en prosjektrekommandasjon basert på lokale notater og primærkildene under.

## Dybde fra stereo

| Kandidat | Rolle | Fordeler for vårt arbeid | Hva må avklares? |
| --- | --- | --- | --- |
| OpenCV SGBM / NVIDIA VPI SGM | Klassisk referanse | Ingen trening; nyttig for å isolere geometri- og dataproblemer | Egne innstillinger, backend, oppløsning og undervannskvalitet |
| NVIDIA Light ESS | Første lærte integrasjonskandidat | Isaac ROS-integrasjon; disparitet og konfidenskart | Kompatibel release på Orin Nano, optisk input, konfidens under vann og samlet ressursbruk |
| LightStereo-S | Alternativ for egen tilpasning | Lett stereoarkitektur med forskningskode | Eksakt variant/checkpoint, eksport, TensorRT-operatører og eventuell undervannsfinjustering |
| RAFT-Stereo / IGEV-Stereo | Sammenligningsmodeller | Etablerte lærte stereometoder | Kostnad og kvalitet på våre data; start med offline-evaluering |
| FoundationStereo | Offline kvalitetsreferanse | Interessant for generalisering til nye scener | Minne, latenstid og undervannsfeil før eventuell drift |

Kilder: [VPI SGM](https://docs.nvidia.com/vpi/algo_stereo_disparity.html), [Light ESS modellkort](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/dnn_stereo_disparity), [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth), [LightStereo](https://arxiv.org/abs/2406.19833), [OpenStereo](https://github.com/XiandaGuo/OpenStereo), [RAFT-Stereo](https://arxiv.org/abs/2109.07547), [IGEV](https://arxiv.org/abs/2303.06615), [FoundationStereo](https://github.com/NVlabs/FoundationStereo).

Light ESS-modellkortet oppgir RGB-input på 480 × 288 og et disparitets- og konfidenskart. Eksakt preprocessing og modellversjon må kontrolleres ved integrasjon. Oppgitte AGX Orin-tall gjelder inferens alene og dokumenterer ikke hele vår pipeline på Orin Nano.

Konfidens er ikke en kalibrert avstandsfeil i millimeter. Sjekk at høy konfidens faktisk korrelerer med lav målefeil under våre vannforhold. LightStereo har ikke nødvendigvis samme konfidensgrensesnitt; etabler egen kvalitetskontroll der det trengs.

## Objekt og orientering

| Metode | Bruk når | Avgrensning |
| --- | --- | --- |
| AprilTag / ArUco | Objekt kan merkes og gripepunkt er kjent | Krever synlig merke, kjent størrelse og validert kameramodell |
| Konturer, farge, linjer og formtilpasning | Objekt/geometri er enkel og miljøet kontrollert | Følsomt for endret lys, bakgrunn og sikt |
| Liten YOLO-segmenteringsmodell | Flere kjente objekter og mer varierende bakgrunn | Trenger egne representative undervannsdata og korrekt maskeregistrering |
| FoundationPose | Oppgaven trenger full objektpose | Krever nødvendige objektdata og brukbar dybde; må ytelsestestes |

Kilder: [AprilTag-prosjektet](https://april.eecs.umich.edu/software/apriltag), [YOLO-segmentering](https://docs.ultralytics.com/tasks/segment), [FoundationPose](https://github.com/NVlabs/FoundationPose).

## Valgregel

Velg den enkleste kandidaten som oppfyller gripeoppgavens posisjonsfeil, orienteringsfeil, dekning og latenstid på de samme holdte-utenfor testopptakene. Treningsdata og testdata deles etter opptaksøkt/scene, slik at nabobilder ikke gir kunstig gode resultater.

Lagre modellnavn, checkpoint, kodecommit, inputoppløsning, preprocessing, presisjon og programvareversjoner sammen med testresultatene. Gjennomgå kode- og vektlisenser før integrasjon; en åpen kodebase innebærer ikke automatisk samme lisens for modellvektene.

Monokulær dybde er ikke førstevalg for metrisk griping her: antatt skala og domeneavhengighet må valideres. Kjent markørstørrelse eller CAD-geometri er en annen situasjon fordi den tilfører målestokk.
