// ###### Config options ################


// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

void asignarValor(std::list<Defense*> defenses, float* valor)
{
	//atributos necesearios para asignar valores
	float dano=0, ataque=0,rango=0, vida=0, dispersion=0;
	int nDefensa=0;

	for(std::list<Defense*>::iterator defensa  = defenses.begin(); defensa != defenses.end(); defensa++)
	{
		dano = (20*(*defensa)->damage);
		ataque = (20*(*defensa)->attacksPerSecond);
		rango = (20*(*defensa)->range);
		vida = (30*(*defensa)->health);
		dispersion = (10*(*defensa)->dispersion);

		valor[nDefensa] = dano + ataque + rango + vida + dispersion;
		nDefensa++;
	}
	valor[0] = 9999999999;
}

void DEF_LIB_EXPORTED selectDefenses(std::list<Defense*> defenses, unsigned int ases, std::list<int> &selectedIDs
            , float mapWidth, float mapHeight, std::list<Object*> obstacles)
{
 
    //Creacion de matriz y vector para guardar distintos valores de la defensa
    float matriz[defenses.size()][ases];
	float valor[defenses.size()];

	//Metodo para la asignacion de los valores
	asignarValor(defenses,valor);

	std::list<Defense*>::iterator defensa = defenses.begin();

	//Algoritmo determinista

	for(int i = 0; i <= ases; i++)
	{
		if(i < (*defensa)->cost)
			matriz[0][i] = 0;
		else
			matriz[0][i] = valor[0];
	}
	++defensa;
	int i = 1;

	

	while(defensa != defenses.end())
	{
		for(int j = 0; j <= ases; ++j)
		{
			if(j < (*defensa)->cost)
				matriz[i][j] = matriz[i-1][j];
			else
				matriz[i][j] = std::max(matriz[i-1][j], matriz[i-1][j-(*defensa)->cost] + valor[i]);
		}
		++i;
		++defensa;
	}

    i = defenses.size()-1;
    int j = ases-1;
    std::list<Defense*>::iterator asignar = defenses.end();
    --asignar;

    //Asignacion de los id segun los valores que le hemos dado a las defensas
    while(i > 0)
    {
    	if(matriz[i][j] != matriz[i-1][j])
    	{
	    	selectedIDs.push_back((*asignar)->id);
	    	j = j - (*asignar)->cost;
    	}	
    	--i;
	    --asignar;	
    }

    if(matriz[0][j] != 0)
    	selectedIDs.push_back((*asignar)->id);
}


