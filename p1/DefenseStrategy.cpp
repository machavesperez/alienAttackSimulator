// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

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

void DEF_LIB_EXPORTED placeDefenses(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
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

