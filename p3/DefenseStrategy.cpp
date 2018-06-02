// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"
#include <vector>
#include <algorithm>

/*#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif*/

using namespace Asedio;

struct celda
{
  int x;
  int y;
  float punt;
  struct celda operator =(const struct celda& c2)
  {
    x = c2.x;
    y = c2.y;
    punt = c2.punt;
    return *this;
  }
};

typedef struct celda sCelda;

bool operator <(const sCelda& c1, const sCelda& c2)
{
  return (c1.punt < c2.punt);
}

void sinPreordenacion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses);
void conPreordenacion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses,int op);
void conMonticulo(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses);
              
//Ordenación
void insercion(sCelda* cells , int i, int j);
void fusion(sCelda* v, int i, int k, int j);
void ordfusion(sCelda* v, int i , int j);
int pivote(sCelda* v, int i , int j);
void ordrapida(sCelda* v, int i, int j);
//Prueba de caja negra   
bool ordenado(sCelda w[]);

void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , List<Object*> obstacles, List<Defense*> defenses) 
{

  float cellWidth = mapWidth / nCellsWidth;
  float cellHeight = mapHeight / nCellsHeight; 


#ifdef PRUEBAS_CAJA_NEGRA
    sCelda celdas[10];
    sCelda w[10];
    for(int i=0;i<10;++i) {
        celdas[i].punt = i;
        celdas[i].x = 0;
        celdas[i].y = 0;
    }
    
    while ( std::next_permutation(celdas, celdas+10)) {
      
      for(int i = 0; i < 10; i++)
      {
        w[i] = celdas[i];
      }
      
        ordfusion(w,0, 10-1);
        if(!ordenado(w))
            std::cout << "Error" << std::endl ;
    }
  
    for(int i=0 ; i < 10 ; ++i) {
        celdas[i].punt = i;
        celdas[i].x = 0;
        celdas[i].y = 0;
    }
    
    while ( std::next_permutation(celdas, celdas+10)) {
    
        for(int i = 0; i < 10; i++)
      {
        w[i] = celdas[i];
      }
      
        ordrapida(w,0,10-1);
        if(!ordenado(w)) 
            std::cout << "Error" << std::endl ;
    }
#endif
  cronometro cA;
  long int r0 = 0;
  cA.activar();
  int error = 0.01/0.001 + 0.01;
  do
  { 
    sinPreordenacion(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
    ++r0;
  }while(cA.tiempo() < error);
  cA.parar();
  cronometro cB;
  long int r1 = 0;
  cB.activar();
  do
  { 
    conPreordenacion(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses,1); //Fusión
    ++r1;
  }while(cB.tiempo() < error);
  cB.parar();
  cronometro cC;
  long int r2 = 0;
  cC.activar();
  do
  { 
    conPreordenacion(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses,2); //Ordenación rápida.
    ++r2;
  }while(cC.tiempo() < error);
  cC.parar();
  cronometro cD;
  long int r3 = 0;
  cD.activar();
  do
  {
    conMonticulo(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
    ++r3; 
  }while(cD.tiempo() < error);
  cD.parar();
  std::cout << (nCellsWidth * nCellsHeight) << '\t' <<cA.tiempo() / r0 << '\t' << cB.tiempo() / r1 << '\t' << cC.tiempo() / r2 << '\t' << cD.tiempo() / r3 << std::endl;
}

void valorCeldaExtratora(float** valorCelda, float lCelda, std::list<Object*> obs, int nCelda)
{

  float centroMapa = ((nCelda/2)*lCelda) + (lCelda/2);
 
  //obstáculo más cercano del centro
  std::list<Object*>::iterator candidato = obs.begin();
  for(std::list<Object*>::iterator objeto = obs.begin(); objeto != obs.end(); objeto++)
  {
    if(sqrt(pow((*objeto)->position.x-centroMapa,2) + pow((*objeto)->position.y-centroMapa,2)) < sqrt(pow((*candidato)->position.x-centroMapa,2) + pow((*candidato)->position.y-centroMapa,2)))
      candidato = objeto;
  }

  //Recorremos todas las celdas del mapa
  for(int i = 0; i < nCelda; i++)
  {
    for(int j = 0; j < nCelda; j++)
    {
      //Calculamos los puntos medio de cada celda
      float puntoMedioY = j*lCelda + lCelda/2;
      float puntoMedioX = i*lCelda + lCelda/2;

      //Gardamos la distancia euclideas de todas las celdas con el obstaculo elegido anteriormente
      float valorDistancia = sqrt(pow((*candidato)->position.x-puntoMedioX,2) + pow((*candidato)->position.y-puntoMedioY,2));
      valorCelda[i][j] = valorDistancia;
    }
  }
}

void valorCeldaDefensas(float** valorCelda, float cellW, std::list<Defense*> defenses, int nCelda)
{
  List<Defense*>::iterator Extractora = defenses.begin();
  for(int i = 0; i < nCelda; i++)
  {
    for(int j = 0; j < nCelda; j++)
    {
      valorCelda[i][j] = 0;

      //Calculasmos los puntos medios de cada celda
      float puntoMedioY = j*cellW + cellW/2;
      float puntoMedioX = i*cellW + cellW/2;
      
      //Distancia euclidea entre la extractora a la celda seleccionada.
      float valorDistancia = sqrt(pow((*Extractora)->position.x-puntoMedioX,2) + pow((*Extractora)->position.y-puntoMedioY,2));
      valorCelda[i][j] = valorDistancia;
    } 
  }
}

bool factibilidad(float puntoMedioX, float puntoMedioY, std::list<Object*> lObjeto, std::list<Defense*> lDefensas, float rad, float TamMap)
{
  bool factible = true;
  
  for(std::list<Object*>::iterator objeto = lObjeto.begin(); objeto != lObjeto.end() && factible; objeto++)
  {
    //Si la distancia euclideas hasta el obstaculo es menor que la suma de los radios
    if((sqrt(pow((*objeto)->position.x-puntoMedioX,2) + pow((*objeto)->position.y-puntoMedioY,2))-(*objeto)->radio-rad) <= 0)
    {
      factible = false;
    }
  }

  for(std::list<Defense*>::iterator defensa = lDefensas.begin(); defensa != lDefensas.end() && factible; defensa++)
  {
    //Si la distancia euclideas hasta alguna defensa es menor que la suma de los radios
    if((sqrt(pow((*defensa)->position.x-puntoMedioX,2) + pow((*defensa)->position.y-puntoMedioY,2))-(*defensa)->radio-rad) <= 0)
      factible = false;
  }

  return factible;
}

void seleccion(float** valorCelda, int *coorX, int *coorY,int nCelda)
{
  float candidato = valorCelda[0][0];
  
  //Inicializamos al infinito
  valorCelda[*coorX][*coorY] = std::numeric_limits<float>::max();
  *coorX = 0;
  *coorY = 0;
  for(int i = 1; i < nCelda; i++)
    for(int j = 1; j < nCelda; j++)
    {
      if(valorCelda[i][j] < candidato)
      {
        *coorX = i;
        *coorY = j;
        candidato = valorCelda[i][j];
      }
    }
}

void sinPreordenacion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses) 

{

  float cellWidth = mapWidth / nCellsWidth; //Calculamos el ancho de una celda
  float cellHeight = mapHeight / nCellsHeight; //Calculamos el alto de una celda
  float** valorCelda = new float*[nCellsHeight]; // Puntuación para las celdas

  //Inicializamos todas las celdas del mapa con el valor nulo
  for(int i=0; i<nCellsHeight; i++)
  {
    valorCelda[i] = new float[nCellsHeight];
    for(int j=0; j<nCellsHeight; j++)
    valorCelda[i][j] = 0;
  }

  List<Defense*>::iterator currentDefense = defenses.begin();

  //Valores de las celda para la colocacion del centro extrator
  valorCeldaExtratora(valorCelda,cellHeight,obstacles,nCellsWidth);
    
  bool asignada = false;

  //Algorizmo voraz para la colocacion de la celda extractora
  while(!asignada)//mientras que no se haya asignado el valor a la celda extratora
  {
    int coorX,coorY;

    seleccion(valorCelda, &coorX, &coorY,nCellsWidth);//Seleccionamos candidatos
    
    //puntos medios
    float puntoMedioY = coorY*cellHeight + cellHeight/2;
    float puntoMedioX = coorX*cellWidth + cellWidth/2;

    //Comprobacion si es posible esa solucion
    if(factibilidad(puntoMedioX,puntoMedioY,obstacles,defenses,(*currentDefense)->radio,mapWidth))
    {   
      (*currentDefense)->position.x = puntoMedioX;
      (*currentDefense)->position.y = puntoMedioY;
      (*currentDefense)->position.z = 0; 
      asignada = true;
    }
  }
  currentDefense++;

  //Valores de celdas para defensas
  valorCeldaDefensas(valorCelda,cellHeight,defenses,nCellsWidth);

  //Algorizmo voraz para la colocacion de las defensas
  while(currentDefense != defenses.end())//mientras que haya defensas
  {
    int coorX,coorY;

    seleccion(valorCelda, &coorX, &coorY,nCellsWidth);//seleccionamos candidatos
    
    //Punto medio
    int puntoMedioY = coorY*cellHeight + cellHeight/2;
    int puntoMedioX = coorX*cellWidth + cellWidth/2;
    
    //Comprobacion si es posible esa solucion
    if(factibilidad(puntoMedioX,puntoMedioY,obstacles,defenses,(*currentDefense)->radio,mapWidth))
    {   
      (*currentDefense)->position.x = puntoMedioX;
      (*currentDefense)->position.y = puntoMedioY;
      (*currentDefense)->position.z = 0; 
      currentDefense++;
    }
  }

  delete [] valorCelda;
}

void valorCeldaExtratoraFu(sCelda* cells, float lCelda, std::list<Object*> obs,int nceldas, int nCelda)
{
   float centroMapa = ((nCelda/2)*lCelda) + (lCelda/2);
 
  //obstáculo más cercano del centro
  std::list<Object*>::iterator candidato = obs.begin();
  for(std::list<Object*>::iterator objeto = obs.begin(); objeto != obs.end(); objeto++)
  {
    if(sqrt(pow((*objeto)->position.x-centroMapa,2) + pow((*objeto)->position.y-centroMapa,2)) < sqrt(pow((*candidato)->position.x-centroMapa,2) + pow((*candidato)->position.y-centroMapa,2)))
      candidato = objeto;
  }

  //Recorremos todas las celdas del mapa
  for(int i = 0; i < nceldas; i++)
  {
    cells[i].punt = 0;
    //Calculamos los puntos medio de cada celda
    float puntoMedioY = cells[i].y*lCelda + lCelda/2;
    float puntoMedioX = cells[i].x*lCelda + lCelda/2;

    //Gardamos la distancia euclideas de todas las celdas con el obstaculo elegido anteriormente
    cells[i].punt += sqrt(pow((*candidato)->position.x-puntoMedioX,2) + pow((*candidato)->position.y-puntoMedioY,2));
  }
  
}

void valorCeldaDefensasFu(sCelda* cells, float cellW, std::list<Defense*> defenses, int nCelda)
{
  List<Defense*>::iterator Extractora = defenses.begin();
  int nCells = nCelda * nCelda;
  for(int i = 0; i < nCells; i++)
  {
     cells[i].punt = 0;

    //Calculasmos los puntos medios de cada celda
    float puntoMedioY = cells[i].y*cellW + cellW/2;
    float puntoMedioX = cells[i].x*cellW + cellW/2;
      
    //Distancia euclidea entre la extractora a la celda seleccionada.
    cells[i].punt = (sqrt(pow((*Extractora)->position.x-puntoMedioX,2) + pow((*Extractora)->position.y-puntoMedioY,2)))*defenses.size(); 
  }
}

void insercion(sCelda* cells , int i, int j)
{
  for(int k = i + 1; k <= j; k++)
  {
    sCelda tmp = cells[k];
    int l = k;
    while((l > i) && (cells[l] < cells[l - 1]))
    {
      cells[l] = cells[l - 1];
      l--;
    }
    cells[l] = tmp;
  }
}

void fusion(sCelda* v, int i, int k, int j)
{
  int n = j - i + 1;
  int p = i;
  int q = k + 1;
  
  sCelda w[n];
  
  for(int l = 0; l <= n; l++)
  {
    if(p <= k && (q > j || v[p].punt <= v[q].punt))
    { 
      w[l] = v[p];
      p++;
    }
    else
    {
      w[l] = v[q];
      q++;
    }
  }
  for(int l = 0; l < n; l++)
  {
    v[i + l] = w[l];
  }
}
void ordfusion(sCelda* v, int i , int j)
{
  int n = j - i + 1;
  if(n <= 2)
  {
    insercion(v,i,j);
  }
  else
  {
    int k = i - 1 + n/2;
    ordfusion(v,i,k);
    ordfusion(v,k+1,j);
    fusion(v,i,k,j);
    
  }
  
}

int pivote(sCelda* v, int i , int j)
{
  int p = i;
  sCelda tmp = v[i];
  sCelda aux;
  for(int k = i + 1; k <= j; k++)
  {
    if(v[k].punt <= tmp.punt)
    {
      p++;
      aux = v[p];
      v[p] = v[k];
      v[k] = aux;
    }
  }
  v[i] = v[p];
  v[p] = tmp;
  return p;
}

void ordrapida(sCelda* v, int i, int j)
{
  int n = j - i + 1;
  if(n <= 2)
  {
    insercion(v,i,j);
  }
  else
  {
    int p = pivote(v,i,j);
    ordrapida(v,i,p-1);
    ordrapida(v,p+1,j);
  }
}

void conPreordenacion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses, int op) {

    float cellWidth = mapWidth / nCellsWidth; //Calculamos el ancho de una celda
    float cellHeight = mapHeight / nCellsHeight; //Calculamos el alto de una celda
    unsigned int nCells = nCellsWidth*nCellsHeight;
    sCelda punt_cells[nCells]; // Puntuación para las celdas

    int nC = 0;
    for(int i = 0; i < nCellsWidth; i++)
    {
      for(int j = 0; j < nCellsHeight; j++)
      {
        punt_cells[nC].x = i;
        punt_cells[nC].y = j;
        punt_cells[nC].punt = 0;
        nC++;
      }
    }
  
    int numDef = 0;
    int cont = 0; //Contador para ir tomando los valores ordenados.
    List<Defense*>::iterator currentDefense = defenses.begin();
    valorCeldaExtratoraFu(punt_cells,cellWidth,obstacles,nCells,nCellsWidth);


    if(op == 1) //Ordenamiento por fusión.
    {
      ordfusion(punt_cells,0,nCells - 1);
    }
    else //Ordenamiento por ordenación rápida.
    {
      ordrapida(punt_cells,0,nCells - 1);
    }
    
  while(numDef == 0 && cont < nCells)
    {
      sCelda selec = punt_cells[cont];
      cont++;
      float cpy = selec.y*cellHeight + cellHeight/2;//Punto medio
    float cpx = selec.x*cellWidth + cellWidth/2;
    float rad = (*currentDefense)->radio; //radio de la defensa actual
      if(factibilidad(cpx,cpy,obstacles,defenses,rad,mapWidth) == true)
      {
        
        (*currentDefense)->position.x = cpx;
        (*currentDefense)->position.y = cpy;
        (*currentDefense)->position.z = 0; 
        currentDefense++;
        numDef++;
      }
    }
      
  valorCeldaDefensasFu(punt_cells,cellWidth,defenses,nCellsWidth);
  
  if(op == 1) //Ordenamiento por fusión.
    {
  ordfusion(punt_cells,0,nCells - 1);
    }
    else //Ordenamiento por ordenación rápida.
    {
      ordrapida(punt_cells,0,nCells - 1);
    }
    
    cont = 0;
    while(currentDefense != defenses.end() && cont < nCells)
    {

      sCelda selec = punt_cells[cont];
      cont++;
      int cpy = selec.y*cellHeight + cellHeight/2;//Punto medio
  int cpx = selec.x*cellWidth + cellWidth/2;

  float rad = (*currentDefense)->radio; //radio de la defensa actual
      if(factibilidad(cpx,cpy,obstacles,defenses,rad,mapWidth) == true)
      {
        
        (*currentDefense)->position.x = cpx;
        (*currentDefense)->position.y = cpy;
        (*currentDefense)->position.z = 0; 
        currentDefense++;
        numDef++;
      }
    }
}
//Finalizan las funciones del método de fusión y ordenación rápida

void valorCeldaExtratoraMon(std::vector<sCelda>& cells, float lCelda, std::list<Object*> obs,int nceldas, int nCelda)
{
   float centroMapa = ((nCelda/2)*lCelda) + (lCelda/2);
 
  //obstáculo más cercano del centro
  std::list<Object*>::iterator candidato = obs.begin();
  for(std::list<Object*>::iterator objeto = obs.begin(); objeto != obs.end(); objeto++)
  {
    if(sqrt(pow((*objeto)->position.x-centroMapa,2) + pow((*objeto)->position.y-centroMapa,2)) < sqrt(pow((*candidato)->position.x-centroMapa,2) + pow((*candidato)->position.y-centroMapa,2)))
      candidato = objeto;
  }

  //Recorremos todas las celdas del mapa
  for(int i = 0; i < nceldas; i++)
  {
    cells[i].punt = 0;
    //Calculamos los puntos medio de cada celda
    float puntoMedioY = cells[i].y*lCelda + lCelda/2;
    float puntoMedioX = cells[i].x*lCelda + lCelda/2;

    //Gardamos la distancia euclideas de todas las celdas con el obstaculo elegido anteriormente
    cells[i].punt += sqrt(pow((*candidato)->position.x-puntoMedioX,2) + pow((*candidato)->position.y-puntoMedioY,2));
  }
  
}

void valorCeldaDefensasMon(std::vector<sCelda>& cells, float cellW, std::list<Defense*> defenses, int nCelda)
{
  List<Defense*>::iterator Extractora = defenses.begin();
  int nCells = nCelda * nCelda;
  for(int i = 0; i < nCells; i++)
  {
     cells[i].punt = 0;

    //Calculasmos los puntos medios de cada celda
    float puntoMedioY = cells[i].y*cellW + cellW/2;
    float puntoMedioX = cells[i].x*cellW + cellW/2;
      
    //Distancia euclidea entre la extractora a la celda seleccionada.
    cells[i].punt = (sqrt(pow((*Extractora)->position.x-puntoMedioX,2) + pow((*Extractora)->position.y-puntoMedioY,2)))*defenses.size(); 
  }
}

void conMonticulo(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses) {

    float cellWidth = mapWidth / nCellsWidth; //Calculamos el ancho de una celda
    float cellHeight = mapHeight / nCellsHeight; //Calculamos el alto de una celda
    unsigned int nCells = nCellsWidth*nCellsHeight;
    std::vector<sCelda> punt_cells; // Puntuación para las celdas
    
    
    sCelda tmp;
    for(int i = 0; i < nCellsWidth; i++)
    {
      for(int j = 0; j < nCellsHeight; j++)
      {
        tmp.x = i;
        tmp.y = j;
        tmp.punt = 0;
        punt_cells.push_back(tmp);
      }
    }
  
  
    int numDef = 0;
    int cont = 0; //Contador para ir tomando los valores ordenados.
    List<Defense*>::iterator currentDefense = defenses.begin();
    valorCeldaExtratoraMon(punt_cells,cellHeight,obstacles,nCells,nCellsWidth);
   
    std::make_heap(punt_cells.begin(),punt_cells.end());
    std::sort_heap(punt_cells.begin(),punt_cells.end());
    
    while(numDef == 0 && cont < nCells)
    {
      sCelda selec = punt_cells[cont];
      cont++;
      float cpy = selec.y*cellHeight + cellHeight/2;//Punto medio
  float cpx = selec.x*cellWidth + cellWidth/2;
  float rad = (*currentDefense)->radio; //radio de la defensa actual
      if(factibilidad(cpx,cpy,obstacles,defenses,rad,mapWidth) == true)
      {
        
        (*currentDefense)->position.x = cpx;
        (*currentDefense)->position.y = cpy;
        (*currentDefense)->position.z = 0; 
        currentDefense++;
        numDef++;
      }
    }
      
  valorCeldaDefensasMon(punt_cells,cellHeight,defenses,nCellsWidth);
  std::make_heap(punt_cells.begin(),punt_cells.end());
  std::sort_heap(punt_cells.begin(),punt_cells.end());
        
    cont = 0;
    while(currentDefense != defenses.end() && cont < nCells)
    {

      sCelda selec = punt_cells[cont];
      cont++;
      int cpy = selec.y*cellHeight + cellHeight/2;//Punto medio
  int cpx = selec.x*cellWidth + cellWidth/2;

  float rad = (*currentDefense)->radio; //radio de la defensa actual
      if(factibilidad(cpx,cpy,obstacles,defenses,rad,mapWidth) == true)
      {
        
        (*currentDefense)->position.x = cpx;
        (*currentDefense)->position.y = cpy;
        (*currentDefense)->position.z = 0; 
        currentDefense++;
        numDef++;
      }
    }
}
//Finalizan las funciones del método de ordenamiento con montículo

bool ordenado(sCelda w[])
{
  bool check = true;
  
  for(int i = 0; (i < 10 - 1) && (check == true); i++)
  {
    if(w[i].punt > w[i+1].punt) {check = false;}
  }
  return check;
}


