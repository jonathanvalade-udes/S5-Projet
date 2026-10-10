"""
test_client.py
Script de test automatisé pour valider le serveur WebSocket (Étape 1).
Simule un client Godot envoyant de la télémétrie et valide les réponses JSON.
"""

import asyncio
import json
import logging
import sys
import websockets

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] (TestClient) %(message)s")
logger = logging.getLogger("TestClient")

URI = "ws://127.0.0.1:8765"


async def run_integration_test():
    logger.info(f"Test 1.1 : Connexion au serveur sur {URI} ...")
    try:
        async with websockets.connect(URI) as ws:
            logger.info("Connexion établie avec succès !")

            # Test 1.2 : Envoi de 30 paquets séquentiels
            logger.info("Test 1.2 : Envoi d'une série de paquets de télémétrie...")
            for i in range(1, 31):
                payload = {
                    "packet_id": i,
                    "distance": 45.0 + (i * 0.5),
                    "line_follower": [0, 0, 1, 0, 0],
                    "velocity": 0.2,
                    "direction": -0.1
                }
                await ws.send(json.dumps(payload))
                raw_response = await asyncio.wait_for(ws.recv(), timeout=2.0)
                response = json.loads(raw_response)

                # Validation des clés requises
                required_keys = ["status", "distance", "line_follower", "velocity", "direction"]
                for key in required_keys:
                    assert key in response, f"Clé manquante dans la réponse : {key}"

                assert isinstance(response["line_follower"], list), "line_follower doit être une liste"
                assert len(response["line_follower"]) == 5, "line_follower doit avoir 5 capteurs"

            logger.info("30 paquets traités et validés avec succès.")

            # Test 1.3 : Envoi de trames limites / malformées
            logger.info("Test 1.3 : Test de résilience (trame invalide et incomplète)...")
            await ws.send("Ceci n'est pas du JSON valide !")
            # Le serveur doit survivre et continuer à répondre à un paquet valide
            await ws.send(json.dumps({"packet_id": 999, "distance": 12.3}))
            raw_response = await asyncio.wait_for(ws.recv(), timeout=2.0)
            res = json.loads(raw_response)
            assert res.get("distance") == 12.3, "Le serveur doit avoir mis à jour la distance"
            logger.info("Test de résilience validé : le serveur ne plante pas.")

    except ConnectionRefusedError:
        logger.error(f"Impossible de se connecter à {URI}. Le serveur est-il démarré ?")
        return False
    except Exception as e:
        logger.error(f"Échec du test : {e}")
        return False

    # Test 1.4 : Reconnexion après fermeture
    logger.info("Test 1.4 : Test de déconnexion et reconnexion...")
    try:
        async with websockets.connect(URI) as ws2:
            await ws2.send(json.dumps({"packet_id": 1000, "velocity": 0.15}))
            res2 = json.loads(await asyncio.wait_for(ws2.recv(), timeout=2.0))
            assert res2.get("velocity") == 0.15
            logger.info("Reconnexion réussie et état validé !")
    except Exception as e:
        logger.error(f"Échec lors de la reconnexion : {e}")
        return False

    logger.info("====================================================")
    logger.info(" TOUS LES TESTS DE L'ÉTAPE 1 ONT RÉUSSI AVEC SUCCÈS ")
    logger.info("====================================================")
    return True


if __name__ == "__main__":
    success = asyncio.run(run_integration_test())
    sys.exit(0 if success else 1)
