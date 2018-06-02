// ###### Config options ################

#define PRINT_PATHS 1

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_PATHS
#include "ppm.h"
#endif

using namespace Asedio;

bool logico(AStarNode* i, AStarNode* j);


Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){
    return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0);
}

void DEF_LIB_EXPORTED calculateAdditionalCost(float** additionalCost
                   , int cellsWidth, int cellsHeight, float mapWidth, float mapHeight
                   , List<Object*> obstacles, List<Defense*> defenses) {
    
    float cellWidth = mapWidth / cellsWidth;
    float cellHeight = mapHeight / cellsHeight;
    float coste,coorY,coorX;
    
    for(int i = 0; i < cellsWidth; i++)
    {
        for(int j = 0; j < cellsHeight; j++)
        {
            coste = 0;
            coorY = j * cellHeight + cellHeight/2;//Punto medio
            coorX = i * cellWidth + cellWidth/2;
        
            List<Defense*>::iterator def = defenses.begin();
            coste = coste + sqrt(pow((*def)->position.x-coorX,2) + pow((*def)->position.y-coorY,2));

            if(sqrt(pow((*def)->position.x-coorX,2) + pow((*def)->position.y-coorY,2)) < 400)
            {
                def++;
                while(def != defenses.end())
                {
                    coste = coste + sqrt(pow((*def)->position.x-coorX,2) + pow((*def)->position.y-coorY,2));
                    def++;
                }
            }
            additionalCost[i][j] = coste;
        }
    }
}

void DEF_LIB_EXPORTED calculatePath(AStarNode* originNode, AStarNode* targetNode
                   , int cellsWidth, int cellsHeight, float mapWidth, float mapHeight
                   , float** additionalCost, std::list<Vector3> &path) {

    float cellWidth = mapWidth / cellsWidth;
    float cellHeight = mapHeight / cellsHeight;
    
    
    std::vector<AStarNode*> abierto;
    std::vector<AStarNode*> cerrado;

    AStarNode* current = originNode;

    current->H = _sdistance(current->position,targetNode->position);
    current->F = current->G+current->H;
    abierto.push_back(current);

    bool encontrado = false;
    std::make_heap(abierto.begin(),abierto.end(),logico);
    int coorX, coorY;
    float distancia;
    
    while(encontrado == false && abierto.size() > 0)
    {
      current = abierto.front();
      std::pop_heap(abierto.begin(),abierto.end(),logico);
      abierto.pop_back();
      cerrado.push_back(current);
      
      
      if(current == targetNode)
      {
        encontrado = true;
      }
        
      else
      {
        for(List<AStarNode*>::iterator it=current->adjacents.begin(); it != current->adjacents.end(); it++)
          if(cerrado.end()==std::find(cerrado.begin(),cerrado.end(),(*it)))
            if(abierto.end()==std::find(abierto.begin(),abierto.end(),(*it)))
            {
              coorX = (*it)->position.x/cellWidth;
              coorY = (*it)->position.y/cellHeight;
              (*it)->parent = current;
              (*it)->G = current->G + _distance(current->position,(*it)->position) + additionalCost[coorX][coorY];
              (*it)->H = _sdistance((*it)->position,targetNode->position);
              (*it)->F = (*it)->G + (*it)->H;
              //Push monticulo
              abierto.push_back(*it);
              std::make_heap(abierto.begin(),abierto.end(),logico);
            }
            else
            {
              distancia = _distance(current->position,(*it)->position);
              if((*it)->G > current->G + distancia)
              {
                (*it)->parent = current;
                (*it)->G = current->G + distancia;
                (*it)->F = (*it)->G + (*it)->H;
                std::sort_heap(abierto.begin(),abierto.end(),logico);
              }
            }
      }
    }
    
    current = targetNode;
    path.push_front(current->position);
    while(current->parent != originNode)
    {
        current = current->parent;
        path.push_front(current->position);
    }
}

bool logico(AStarNode* abierto, AStarNode* cerrado)
{
    return (abierto->F > cerrado->F);
}
