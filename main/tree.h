#ifndef ARVORE_H
#define ARVORE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum EvalType {
  LITERAL = 0,
  SUM,
  SUB,
  MUL,
  DIVI,
} EvalType;

typedef struct Info {
  uint32_t valor;
  EvalType tipo;
} Info;

typedef struct arvore {
   Info info;
   struct arvore *esq;
   struct arvore *dir;
} Arvore;

extern Arvore* cria_arv_vazia(void);
extern Arvore* constroi_arv(Info info, Arvore *e, Arvore *d);
extern bool verifica_arv_vazia(Arvore *a);
extern void arv_libera(Arvore* a);
/* extern bool pertence_arv(Arvore *a,  Info info); */
extern int conta_nos(Arvore *a);
extern int calcula_altura_arvore(Arvore *a);
extern int conta_nos_folha(Arvore *a);
extern void in_imprime_arv(Arvore *a);


#endif /* ARVORE_H */
