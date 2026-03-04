# Analisi codice AEROSTATE

## Panoramica
- Applicazione desktop in **PySide6 + VTK** con modello dati interno (`Project`, `WingModel`, `BayModel`, `SectionModel`) e serializzazione JSON `.aer`.
- Export AVL separato in `avl_export.py`.
- Utility aero e integrazione XFOIL in moduli dedicati (`aero_utils.py`, `xfoil_bridge.py`).

## Punti forti
1. **Modello dati chiaro** con dataclass e default robusti.
2. **Pipeline geometria → export** ben strutturata.
3. **Sistema vincoli tra bay** flessibile con fallback iterativo in presenza di cicli.

## Rischi principali
1. **`main_app.py` monolitico** (UI + business logic + IO nello stesso file), con impatto su manutenibilità e testabilità.
2. **Gestione errori troppo permissiva** in alcuni punti (`except Exception: pass`), con rischio di nascondere errori reali.
3. **Bridge XFOIL senza timeout/controllo ritorno**: possibile blocco o error handling incompleto.

## Raccomandazioni prioritarie
1. Estrarre moduli separati (`models.py`, `serialization.py`, `constraints.py`, `ui_mainwindow.py`).
2. Sostituire i `pass` silenziosi con logging/feedback esplicito.
3. Rafforzare `run_xfoil_polar` con timeout e controllo `returncode`.
4. Unificare la logica dei vincoli in un unico “engine” riusabile da UI ed export.
5. Introdurre test minimi su:
   - round-trip serializzazione `.aer`,
   - catene/cicli di vincoli,
   - parsing polar con input non puliti.
