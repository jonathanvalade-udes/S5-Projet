# Shared Headers

## Description

Ce paquet est partagé entre le client et la voiture.

Elle permet de definir le moyen de communication entre les services et via le client par TCP.


## USAGE

Une fonction de serialization/deserialization est nécessaire pour utiliser les paquets.
Un paquet typique possède cette structure :

```
|----PACKET HEADER----|
|                     |
|       PAYLOAD       |
|                     |
|         CRC         |
|---------------------|
```
La taille du payload est variable dépendament si c'est une commande ou de la télémétrie. Cette taille est spécifié dans le header.

Un CRC 32 bits est calculé de manière optionnelle mais recommendé entre le client et la voiture. Si le CRC est utilisé et qu'il est invalide, le client devrait rejeter le paquet et demander un renvoie si FIRE_AND_FORGET n'est pas mis à 1.

On peut traduire ce qui est envoyé de cette manière :
```cpp
uint8_t buffer[sizeof(PacketHeader) + sizeof(payload) + sizeof(PacketCRC)];
```
> [!WARNING]
> Afin de simplifier le calcule du paquet et d'éviter le désalignement de mémoire, il est demandé que la taille finale du payload soit déjà alignée 32 bits.

## Réponses

Le système de réponse pour tout message demandant une réponse ou qui n'est pas FIRE_AND_FORGET doit retourner un des types de réponses disponible dans le premier 32 bits du payload. Le numéro de séquence doit rester le même pour que le système relai l'information au bon service.

## Télémétries

Un paquet de télémetries rassemblant tout les paquets du système est disponible pour un envoie d'une snapshot vers le client.


## example de Serializer

int serializePacket(uint8_t* buffer, const PacketHeader& header, const uint8_t* payload, size_t payloadLen){
    constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
    if(payloadLen % 4 != 0) return -1;
    std::memcpy(buffer, &header, HEADER_SIZE);
    std::memcpy(buffer+HEADER_SIZE, payload, payloadLen);
    
    if(header.flags & shared::headers::flags::CRC_ENABLED){
        PacketCRC crc = crcCalc(buffer, HEADER_SIZE+payloadLen);
        std::memcpy(buffer+HEADER_SIZE+payloadLen, &crc, 4);
    }

    return 1;
}