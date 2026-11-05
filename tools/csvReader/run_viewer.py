#!/usr/bin/env python3
"""
Serveur et lanceur CLI pour l'outil de relecture et modélisation de télémétrie FastDrink / PiCar-S.
Lance un serveur HTTP local et ouvre automatiquement la page dans le navigateur.
Usage :
    python3 run_viewer.py
    python3 run_viewer.py /chemin/vers/mon_fichier.csv
    python3 run_viewer.py --port 8085
"""

import sys
import os
import argparse
import http.server
import socketserver
import webbrowser
import shutil
import urllib.parse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

class TelemetryHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=SCRIPT_DIR, **kwargs)

    def end_headers(self):
        # Désactiver le cache pour garantir les rechargements instantanés des logs
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        super().end_headers()

def main():
    parser = argparse.ArgumentParser(description="Visualiseur & Modélisateur Télémétrie FastDrink PiCar-S")
    parser.add_argument("csv_file", nargs="?", default=None, help="Fichier CSV de télémétrie à charger directement")
    parser.add_argument("--port", type=int, default=8080, help="Port HTTP local (défaut: 8080)")
    parser.add_argument("--no-browser", action="store_true", help="Ne pas ouvrir le navigateur automatiquement")
    args = parser.parse_args()

    # Si un fichier CSV externe est spécifié, s'assurer qu'il est accessible ou le copier localement
    if args.csv_file:
        csv_path = os.path.abspath(args.csv_file)
        if not os.path.isfile(csv_path):
            print(f"Erreur : Le fichier CSV '{csv_path}' n'existe pas.")
            sys.exit(1)
        
        target_csv = os.path.join(SCRIPT_DIR, "sample_telemetry.csv")
        try:
            shutil.copyfile(csv_path, target_csv)
            print(f"✓ Fichier CSV importé pour le visualiseur : {os.path.basename(csv_path)}")
        except Exception as e:
            print(f"Attention : Impossible de copier le CSV ({e}), ouverture du visualiseur par défaut.")

    port = args.port
    # Tentative d'allocation de port (incrément si occupé)
    max_retries = 10
    httpd = None
    for attempt in range(max_retries):
        try:
            socketserver.TCPServer.allow_reuse_address = True
            httpd = socketserver.TCPServer(("", port), TelemetryHandler)
            break
        except OSError:
            port += 1

    if not httpd:
        print(f"Erreur : Impossible de démarrer le serveur sur les ports {args.port} à {port}")
        sys.exit(1)

    url = f"http://localhost:{port}/index.html"
    print("=" * 65)
    print("  🚀 VISUALISEUR & MODÉLISATEUR TÉLÉMÉTRIE FASTDRINK ROBOT")
    print("=" * 65)
    print(f"Serveur local démarré : {url}")
    print("Appuyez sur Ctrl+C pour arrêter le serveur.")
    print("=" * 65)

    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nArrêt du serveur.")
    finally:
        httpd.server_close()

if __name__ == "__main__":
    main()
