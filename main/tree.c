#include "tree.h"


Arvore* cria_arv_vazia(void) {
   return NULL;
}

Arvore* constroi_arv(Info info, Arvore *e, Arvore *d) {
   Arvore *no = (Arvore*)malloc(sizeof(Arvore));
   no->info = info;
   no->esq = e;
   no->dir = d;
   return no;
}

bool verifica_arv_vazia(Arvore *a) {
   return (a == NULL);
}

void arv_libera(Arvore* a) {
   if (!verifica_arv_vazia(a)) {
      arv_libera (a->esq);
      arv_libera (a->dir);
      free(a);
   }
}

//========= Exercício 1 - pré-ordem ====
void pre_imprime_arv(Arvore *a) {
    if (!verifica_arv_vazia(a)) {
        if (a->info.tipo == LITERAL) { 
          printf("%lu\n", a->info.valor);
        } else {
	  printf("%c\n", (char)a->info.valor);
	}
        pre_imprime_arv(a->esq);
        pre_imprime_arv(a->dir);
    }
}

//========= Exercício 1 - in-ordem ====
void in_imprime_arv(Arvore *a) {
    if (!verifica_arv_vazia(a)) {
        in_imprime_arv(a->esq);
	if (a->info.tipo == LITERAL) { 
          printf("%lu\n", a->info.valor);
        } else {
	  printf("%c\n", (char)a->info.valor);
	}
	
        in_imprime_arv(a->dir);
    }
}

//========= Exercício 1 - pós-ordem ====
void pos_imprime_arv(Arvore *a) {
    if (!verifica_arv_vazia(a)) {
        pos_imprime_arv(a->esq);
        pos_imprime_arv(a->dir);
	if (a->info.tipo == LITERAL) { 
          printf("%lu\n", a->info.valor);
	} else {
	  printf("%c\n", (char)a->info.valor);
	}
    }
}

/* //========= Exercício 2 - pertence ==== */
/* bool pertence_arv(Arvore *a, Info info) { */
/*     if (!verifica_arv_vazia(a)) { */
/*         if (a->info == info) return true; */
/*         return pertence_arv(a->esq, info) || pertence_arv(a->dir, info); */
/*     } else return false; */
/* } */

//========= Exercício 3 - conta nós ====
int conta_nos (Arvore *a) {
    if (!verifica_arv_vazia(a)) {
        return conta_nos(a->esq) + conta_nos(a->dir) + 1;
    } else return 0;
}

//========= Exercício 4 - calcula altura ====
int calcula_altura_arvore(Arvore *a) {
    int c = 0;
    int d = 0;
    if (!verifica_arv_vazia(a)) {
        if (!verifica_arv_vazia(a->esq)) {
            c = calcula_altura_arvore(a->esq);
            return ++c;
        }
        if (!verifica_arv_vazia(a->dir)) {
            d = calcula_altura_arvore(a->dir);
            return ++d;
        }

    } else return -1;

    if (c >= d) {
        return c;
    } else {
        return d;
    }
}
/* solucao mais simples
int calcula_altura_arvore (Arvore *a) {
    if(!verifica_arv_vazia(a)) {
        int c = calcula_altura_arvore(a->esq);
        int d = calcula_altura_arvore(a->dir);
        if(c > d)
            return c + 1;
        else
            return d + 1;
    }

    return -1;
}
*/

//========= Exercício 5 - conta folhas ====
int conta_nos_folha(Arvore *a) {
    int c = 0;
    if (!verifica_arv_vazia(a)) {
        if (verifica_arv_vazia(a->esq) && verifica_arv_vazia(a->dir))
            return ++c;
        return conta_nos_folha(a->esq) + conta_nos_folha(a->dir);
    } else return 0;
}

/* int main (int argc, char *argv[]) { */

/*    Arvore *a = constroi_arv ('a', */
/*        constroi_arv('b', */
/*        cria_arv_vazia(), */
/*        constroi_arv('d', cria_arv_vazia(), cria_arv_vazia()) */
/*      ), */
/*      constroi_arv('c', */
/*        constroi_arv('e',cria_arv_vazia(),cria_arv_vazia()), */
/*        constroi_arv('f',cria_arv_vazia(),cria_arv_vazia()) */
/*      ) */
/*    );	 */

/*    //========= Exercício 1 - pré-ordem ==== */
/*    printf("//// pré-ordem ////\n"); */
/*    pre_imprime_arv (a); */

/*    //========= Exercício 1 - in-ordem ==== */
/*    printf("//// in-ordem ////\n"); */
/*    in_imprime_arv (a); */

/*    //========= Exercício 1 - pós-ordem ==== */
/*    printf("//// pós-ordem ////\n"); */
/*    pos_imprime_arv (a); */

/*    //========= Exercício 2 - pertence ==== */
/*    bool pertence = pertence_arv (a, 'a'); */
/*    printf("char `a` pertence a arvore `a`: %d\n", pertence); */

/*    //========= Exercício 3 - pertence ==== */
/*    int nos = conta_nos(a); */
/*    printf("total de nos da arvore `a`: %d\n", nos); */

/*    //========= Exercício 4 - pertence ==== */
/*    int altura = calcula_altura_arvore(a); */
/*    printf("altura da arvore `a`: %d\n", altura); */

/*    //========= Exercício 5 - conta folhas ==== */
/*    int nos_folha = conta_nos_folha(a); */
/*    printf("quantidade de nos folha da arvore `a`: %d\n", nos_folha); */

/*    arv_libera (a); */

/*    return 0; */
/* } */
