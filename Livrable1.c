#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <float.h>

// STRUCTURE DE DONNÉES — CONTRAINTE §1 //

struct Drone {
    int   id;
    float x;
    float y;
    float z;
};
typedef struct Drone Drone;

// RÉSULTAT : paire de drones la plus proche //
typedef struct {
    Drone *a;       // Pointeur vers le premier drone  //
    Drone *b;       // Pointeur vers le second drone   //
    float  distance;
} PaireLaPlusProche;

// FONCTIONS UTILITAIRES //

/* distance_euclidienne_3D
 * Calcule la distance euclidienne entre deux drones en 3D.
 * Reçoit des pointeurs — pas d'indexation par crochets.
 * Complexité : O(1) 
*/
static float distance_euclidienne_3D(const Drone *a, const Drone *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* min_float
 * Retourne le minimum de deux flottants.
*/
static float min_float(float a, float b)
{
    return (a < b) ? a : b;
}

/* COMPARATEURS POUR qsort — ARITHMÉTIQUE DE POINTEURS DANS LES CASTS
   qsort() passe des void* que l'on caste en Drone* et déréférence avec *
   pour accéder aux champs (pas de crochets).
*/

// Tri des Drone par coordonnée x //
static int cmp_x(const void *pa, const void *pb)
{
    const Drone *a = (const Drone *)pa;   // cast void* → Drone* //
    const Drone *b = (const Drone *)pb;
    // Déréférencement de pointeur : a->x équivaut à (*a).x //
    if (a->x < b->x) return -1;
    if (a->x > b->x) return  1;
    return 0;
}

// Tri des pointeurs Drone* par coordonnée y (pour la bande centrale) //
static int cmp_ptr_y(const void *pa, const void *pb)
{
    // pa et pb sont des Drone** → on les caste et déréférence une fois //
    const Drone *a = *(const Drone **)pa;
    const Drone *b = *(const Drone **)pb;
    if (a->y < b->y) return -1;
    if (a->y > b->y) return  1;
    return 0;
}

/* CAS DE BASE — Résolution naïve pour petits sous-ensembles (n ≤ 3)
   Pour n ≤ 3 drones, on compare toutes les paires directement.
   Arithmétique de pointeurs : *(debut + i), *(debut + j)
*/
static PaireLaPlusProche base_brute_force(Drone *debut, int n)
{
    PaireLaPlusProche res;
    res.distance = FLT_MAX;
    res.a = NULL;
    res.b = NULL;

    int i, j;
    for (i = 0; i < n - 1; i++) {
        for (j = i + 1; j < n; j++) {
            // Accès par arithmétique de pointeurs — JAMAIS debut[i] //
            Drone *di = debut + i;   // adresse du drone i //
            Drone *dj = debut + j;   // adresse du drone j //
            float d = distance_euclidienne_3D(di, dj);
            if (d < res.distance) {
                res.distance = d;
                res.a = di;
                res.b = dj;
            }
        }
    }
    return res;
}

/* BANDE CENTRALE — Vérification des voisins dans la bande de largeur 2δ
   On ne compare chaque drone qu'avec ses 7 suivants dans la bande (triée
   sur y). Résultat géométrique : au plus 7 comparaisons suffisent.

   Paramètres :
     bande   : tableau de pointeurs vers les drones dans la bande
     taille  : nombre d'éléments dans la bande
     delta   : distance minimale courante
     meilleur: résultat à mettre à jour si on trouve mieux
*/
static void verifier_bande(Drone **bande, int taille, float delta,
                            PaireLaPlusProche *meilleur)
{
    // Tri de la bande sur y — arithmétique de pointeurs dans le comparateur //
    qsort(bande, (size_t)taille, sizeof(Drone *), cmp_ptr_y);

    int i, j;
    for (i = 0; i < taille; i++) {
        Drone *di = *(bande + i);   // déréférencement du pointeur de pointeur //

        for (j = i + 1; j < taille; j++) {
            Drone *dj = *(bande + j);

            /* Optimisation clé : si dy > delta, aucun drone plus loin sur y
               ne peut être à distance < delta → on arrête la boucle interne */
            if (dj->y - di->y >= delta)
                break;

            float d = distance_euclidienne_3D(di, dj);
            if (d < meilleur->distance) {
                meilleur->distance = d;
                meilleur->a = di;
                meilleur->b = dj;
            }
        }
    }
}

/* CŒUR DE L'ALGORITHME — Diviser-pour-Régner Récursif
   Précondition : le sous-tableau [debut, debut+n) est déjà trié sur x.
   Retourne la paire de drones la plus proche dans ce sous-tableau.
   Complexité : T(n) = 2·T(n/2) + O(n log n)  →  O(n log² n) total
*/
static PaireLaPlusProche closest_pair_rec(Drone *debut, int n)
{
    // ── Cas de base ─────────────────────────────────── //
    if (n <= 3)
        return base_brute_force(debut, n);

    // ── Division ────────────────────────────────────── //
    int milieu_idx = n / 2;
    Drone *milieu  = debut + milieu_idx;   // pointeur vers l'élément médian //
    float  x_mid   = milieu->x;

    // ── Conquête récursive ───────────────────────────── //
    PaireLaPlusProche gauche = closest_pair_rec(debut,  milieu_idx);
    PaireLaPlusProche droite = closest_pair_rec(milieu, n - milieu_idx);

    // δ = meilleure distance trouvée jusqu'ici //
    PaireLaPlusProche meilleur;
    if (gauche.distance <= droite.distance)
        meilleur = gauche;
    else
        meilleur = droite;

    float delta = meilleur.distance;

    // ── Construction de la bande centrale ───────────── //
    /*
     * On alloue un tableau de pointeurs pour la bande.
     * Taille maximale : n pointeurs (cas dégénéré où tous les drones
     * sont dans la bande). On utilise un pointeur de pointeurs — 
     * arithmétique de pointeurs pour remplir la bande.
     */
    Drone **bande = (Drone **)malloc((size_t)n * sizeof(Drone *));
    if (!bande) {
        fprintf(stderr, "[ERREUR] malloc bande échoué\n");
        return meilleur;
    }

    int taille_bande = 0;
    int i;
    for (i = 0; i < n; i++) {
        Drone *d = debut + i;   // arithmétique de pointeurs : pas de [] //
        if (fabsf(d->x - x_mid) < delta) {
            // On stocke l'adresse du drone dans la bande //
            *(bande + taille_bande) = d;   // écriture via arithmétique pointeurs //
            taille_bande++;
        }
    }

    // ── Vérification de la bande ────────────────────── //
    if (taille_bande > 1)
        verifier_bande(bande, taille_bande, delta, &meilleur);

    free(bande);
    return meilleur;
}

/* POINT D'ENTRÉE PUBLIC DE L'ALGORITHME
   Fonction principale appelée par le module de sécurité.

   Paramètres :
     essaim : pointeur vers le premier drone de l'entrepôt mémoire
     n      : nombre de drones (10 000 en production)

   Retourne la paire de drones la plus proche.
*/
PaireLaPlusProche trouver_paire_la_plus_proche(Drone *essaim, int n)
{
    if (!essaim || n < 2) {
        PaireLaPlusProche vide = { NULL, NULL, FLT_MAX };
        return vide;
    }

    /*
     * Tri préalable sur x — O(N log N)
     * qsort accède aux éléments via le comparateur cmp_x qui utilise
     * l'arithmétique de pointeurs en interne.
     */
    qsort(essaim, (size_t)n, sizeof(Drone), cmp_x);

    // Appel récursif sur l'ensemble du tableau trié //
    return closest_pair_rec(essaim, n);
}

/* GÉNÉRATION DE L'ESSAIM — Remplissage aléatoire du tas
   Remplit le tableau dynamique avec des drones à positions aléatoires.
   Navigation exclusive par arithmétique de pointeurs.
*/
static void generer_essaim(Drone *essaim, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        Drone *d = essaim + i;   // décalage de i × sizeof(Drone) octets //
        d->id = i;
        d->x  = ((float)rand() / RAND_MAX) * 1000.0f;
        d->y  = ((float)rand() / RAND_MAX) * 1000.0f;
        d->z  = ((float)rand() / RAND_MAX) * 1000.0f;
    }
}

/* VALIDATION — Vérification naïve sur un petit échantillon
   Pour N_TEST drones seulement (O(N²) acceptable), on vérifie que notre
   algorithme renvoie bien la bonne réponse.
*/
static void valider_petit_essaim(int n_test)
{
    printf("\n[VALIDATION] Vérification naïve sur %d drones...\n", n_test);

    Drone *essaim_test = (Drone *)malloc((size_t)n_test * sizeof(Drone));
    if (!essaim_test) {
        fprintf(stderr, "[ERREUR] malloc validation échoué\n");
        return;
    }

    srand(42);   // Graine fixe pour reproductibilité //
    generer_essaim(essaim_test, n_test);

    /* Résultat naïf O(N²) */
    float   min_naive = FLT_MAX;
    Drone  *na = NULL, *nb = NULL;
    int i, j;
    for (i = 0; i < n_test - 1; i++) {
        for (j = i + 1; j < n_test; j++) {
            float d = distance_euclidienne_3D(essaim_test + i, essaim_test + j);
            if (d < min_naive) {
                min_naive = d;
                na = essaim_test + i;
                nb = essaim_test + j;
            }
        }
    }

    // Résultat algorithmique O(N log² N) //
    // On recrée le tableau car trouver_paire_la_plus_proche trie sur place //
    Drone *essaim_algo = (Drone *)malloc((size_t)n_test * sizeof(Drone));
    if (!essaim_algo) {
        free(essaim_test);
        return;
    }
    int k;
    for (k = 0; k < n_test; k++)
        *(essaim_algo + k) = *(essaim_test + k);

    PaireLaPlusProche res = trouver_paire_la_plus_proche(essaim_algo, n_test);

    printf("  Naïf      : distance = %.6f  (drones %d et %d)\n",
           min_naive, na->id, nb->id);
    printf("  Algo DC   : distance = %.6f  (drones %d et %d)\n",
           res.distance, res.a->id, res.b->id);

    float ecart = fabsf(res.distance - min_naive);
    if (ecart < 1e-3f)
        printf("  RÉSULTAT  : [OK] Les deux distances concordent (écart=%.2e)\n", ecart);
    else
        printf("  RÉSULTAT  : [ERREUR] Discordance détectée ! (écart=%.4f)\n", ecart);

    free(essaim_test);
    free(essaim_algo);
}

//  PROGRAMME PRINCIPAL //

int main(void)
{
    const int N = 10000;   // Taille de l'essaim en production //

    printf("=== Systeme de Detection de Collision - Essaim UAV (N=%d) ===\n\n", N);

    // Phase 1 : Validation de l'algorithme sur petit ensemble //
    valider_petit_essaim(200);

    // Phase 2 : Simulation temps reel sur N=10 000 drones //
    printf("\n[PRODUCTION] Allocation de %d drones sur le tas...\n", N);

    /*
     * Contrainte §2 : allocation unique via malloc — entrepôt continu.
     * `essaim` est le seul pointeur vers la base du tableau.
     */
    Drone *essaim = (Drone *)malloc((size_t)N * sizeof(Drone));
    if (!essaim) {
        fprintf(stderr, "[ERREUR CRITIQUE] Echec d'allocation memoire (%zu octets requis)\n",
                (size_t)N * sizeof(Drone));
        return EXIT_FAILURE;
    }

    printf("[PRODUCTION] Entrepot memoire : %zu octets alloues a l'adresse %p\n",
           (size_t)N * sizeof(Drone), (void *)essaim);

    // Génération des positions radar (simulées) //
    srand((unsigned int)time(NULL));
    generer_essaim(essaim, N);

    // Phase 3 : Detection //
    printf("[DETECTION] Lancement de l'algorithme Diviser-pour-Regner...\n");

    clock_t t_debut = clock();
    PaireLaPlusProche resultat = trouver_paire_la_plus_proche(essaim, N);
    clock_t t_fin = clock();

    double duree_ms = 1000.0 * (double)(t_fin - t_debut) / CLOCKS_PER_SEC;

    // Phase 4 : Rapport de sécurité //
    printf("\n--- RAPPORT DE SECURITE ---\n");
    printf("Paire critique detectee :\n");
    printf("  Drone A : ID=%-5d  pos=(%.2f, %.2f, %.2f)\n",
           resultat.a->id, resultat.a->x, resultat.a->y, resultat.a->z);
    printf("  Drone B : ID=%-5d  pos=(%.2f, %.2f, %.2f)\n",
           resultat.b->id, resultat.b->x, resultat.b->y, resultat.b->z);
    printf("  Distance minimale : %.4f m\n", resultat.distance);
    printf("  Temps de calcul   : %.3f ms\n", duree_ms);
    printf("  Complexite        : O(N log2(N)) ~ O(%d)\n", (int)(N * 13.0 * 13.0));
    printf("  Statut            : MANOEUVRE D'EVITEMENT DECLENCHEE\n");

    // Libération de l'entrepôt mémoire //
    free(essaim);
    essaim = NULL;   // Bonne pratique : évite le dangling pointer //

    printf("\n[INFO] Memoire liberee. Systeme pret pour la prochaine frame.\n");
    return EXIT_SUCCESS;
}
