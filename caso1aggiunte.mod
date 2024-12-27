/*********************************************
 * OPL 12.10.0.0 Model
 * Author: ricca
 * Creation Date: Dec 20, 2019 at 8:56:19 AM
 *********************************************/

execute timeTermination {
    cplex.tilim = 3600*4;    
	//cplex.tilim = 60 * 10;
}    

int n = ...;
int c = ...;
int d = ...;
int t = ...;
int terms_num = 3; // Numero di termini in f.o.
int table_terms = 10; // Numero di righe per la tabella excel
float lambda_step = 1/10; // Ogni lambda viene diminuito di un certo step

range N = 1..n;
range C = 1..c;
range D = 1..d;
range T = 0..t;
range terms = 0..terms_num-1;
range TOT_terms = 0..4;
								
int Emax = 48;
int Emin = 4;
int E_drone = 3; // Circa 4Wh per km			
float E_truck = 1150; // Circa 1.15 kWh per km


tuple tappa {
	float lat;
	float lon;
}

tuple cliente {
	float lat;
	float lon;
}

tuple arco {
	int i;
	int j;
	int cost; // In Wh -> Energia
	float dist; // In Km -> Distanza
}

{arco} A1;
{arco} A2;

{tappa} tappe = ...;
{cliente} clienti = ...;

range cols = 1..8;
string colors[cols] = ...;
float t_start;

//float dist_tappe;
float dist[N][N]=...; // Distanze fra due tappe in km
int dist2[N][C]; // Distanze fra nodo e cliente
int reachable[C]; // 1 se cliente raggiungibile 0 altrimenti

float lambda[TOT_terms] = ...; // Coefficienti da moltiplicare
int param[TOT_terms] = ...; // Termini per normalizzare

int Nc;

// Trasformazione coordinate con la formula di Haversine
execute {
	function degToRad(degree) {
 		return Math.PI/180 * degree
	}

	var i = 1, j = 1;
	var R = 6371; // Raggio della Terra in km
	
	
	for(var ii in N) {
		for(var jj in N) {
			if(jj > ii) {		
				dist[jj][ii] = dist[ii][jj];
			}
		}
	}
	
	for(var ii in N) {
		for(var jj in N) {
			A1.add(ii, jj, E_truck*Opl.ceil(dist[ii][jj]), Opl.ceil(dist[ii][jj])); // Senza Opl.ceil() -> distanze precise ma il modello � molto lento
		}
	}
	
	for(var cl in C) {
		reachable[cl] = 0;
	}
	
	i = 1;
	j = 1;
	
	// Distanze tra tappe e clienti
	for(var t in tappe) {
		for(var cl in clienti) {		
			var latr = degToRad(cl.lat - t.lat);
			var lonr = degToRad(cl.lon - t.lon);
			var lat1r = degToRad(t.lat);
			var lat2r = degToRad(cl.lat);
			
			var a = Math.sin(latr/2) * Math.sin(latr/2) + Math.sin(lonr/2) * Math.sin(lonr/2) * Math.cos(lat1r) * Math.cos(lat2r); 
			var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
			dist2[i][j] = Opl.round(R * c * 10); // Calcolo in km
			
			if(E_drone*2*dist2[i][j] <= Emax-Emin && dist2[i][j] != 0) {
				reachable[j] = 1;
				A2.add(i, j, 2*E_drone*dist2[i][j], 2*dist2[i][j]);
 			}		
 			
			j++;	
		}
		j=1;		
		i++;
	}
	
	Nc = 0;
	for(var cl in C) {
		if(reachable[cl] == 1)
			Nc++;
	}
	
	var date = new Date();
	t_start = date.getTime();
}

dvar int x[A2][D][T] in 0..1; // 1 se Cliente C servito dal drone D dal nodo N al tempo T, 0 altrimenti
dvar int y[N][T] in 0..1; // 1 se il truck si trova al nodo N al tempo T, 0 altrimenti
dvar int z[A1][T] in 0..1; // 1 se il truck si sposta dal nodo N1 al nodo N2 al tempo T-1
dvar float+ e[D][T]; // Energia del drone D al tempo T
dvar float+ f[A1][T]; // Variabile di flusso dal nodo N1 al nodo N2 al tempo T

dvar int w[D][T] in 0..1; // 1 se il drone D si ricarica al tempo T, 0 altrimenti
dvar int q[D][T] in 0..1; // 1 se il drone D al tempo T serve almeno 1 cliente, 0 altrimenti
dvar float+ r[D][T];

// Variabili per il risultato
dvar int+ res;
dvar int+ truck_km;
dvar int+ truck_en;
dvar int+ drone_km;
dvar int+ drone_en;
dvar int+ drone_energy_level;

dvar int+ c_sat; // Customers satisfied

dvar int+ k[T]; // Numero di clienti serviti al tempo T

float M = 1.0 * maxint;
int W = 5000;
dvar int+ tmax;

// New variables
dvar float+ dmax[T]; // maximum mission time of UAVs in time slot t
dvar float+ I[D][T];
float TS = 0.5; // time slot duration (in hours)
dvar float+ obj_dmax;

//nuove nuove variabili
dvar int m1[D][T] in 0..1;
dvar int m2[D][T] in 0..1;


minimize
  	lambda[0]*(sum(a in A1, ti in T)
  	  a.cost * z[a][ti])/param[0]
  + lambda[1]*(sum(a in A2, di in D, ti in T) 
  	  a.cost * x[a][di][ti])/param[1];
//  - lambda[2]*(sum(ti in T: ti>0)
  //	  dmax[ti]);//param[2];
  //- lambda[2]*(sum(di in D, ti in T)
  //		e[di][ti])/param[2];
  	//+ sum(di in D, ti in T)
  		//w[di][ti];
  	//+ tmax*30 // Minimizzazione tempo di consegna [MinTime]
  	//- sum(di in D, a in A2, ti in T:ti>0) // se [customers_MAX]
  		//W*x[a][di][ti];

/*
maximize
  	sum(a in A1, ti in T)
  		a.cost * z[a][ti];
  	//sum(a in A2, di in D, ti in T) 
  	//  	a.cost * x[a][di][ti];
  	//sum(ti in T: ti>0)
  	//	dmax[ti]/1);
*/

subject to {

	forall(ni in N) {
		vnr1:
		sum(a in A1, ti in T: a.i == ni && a.i != a.j) z[a][ti] <= 1;	
		
		vnr2:
		sum(a in A1, ti in T: a.j == ni && a.i != a.j) z[a][ti] <= 1;
	}
	
	/*
	// Add if CASE-2
	// Se modello [MinTime]
	forall(ti in T, di in D, a in A2: ti>0) {	
		ctwt:
		tmax >= ti-M*(1- x[a][di][ti]);
	}
		*/
	forall(ti in T: ti>0) {
		ctk1:
		k[ti] == k[ti-1] + sum(a in A2, di in D) x[a][di][ti];	
	}
	
	forall(ti in T) {
		ctk2:
		sum(a in A1: a.j==1) z[a][ti] >= 1 + (k[ti] - Nc);
		
		ctk3:
		sum(a in A1:a.i!=1 && a.j==1) z[a][ti] <= 1 - (k[ti] - Nc);
	}
	//----------------------------------------------------------//
	
	// <= se modello [customers_MAX]
	forall(j in C) {
		ct2:
		sum(ti in T, di in D, a in A2: ti>0 && j==a.j)
		  x[a][di][ti] == reachable[j];	
	}
	
	//forall(di in D, ti in T: ti>0) {
	//	ct3:
	//	sum(a in A2)
	//	  x[a][di][ti] <= 1;
	//}
	
	forall(di in D, ti in T, a in A2: ti>0 && ti<=t) {
		ct4:
		x[a][di][ti] <= y[a.i][ti];
	}
	
	forall(i in N, ti in T: ti>0) {	
		ct5:
		y[i][ti] == sum(a in A1: a.j==i) z[a][ti-1];
	}		
	
	ct6:
	sum(a in A1: a.i==1 && a.j!=1)
		z[a][0] == 1;
	
	ctnew:
	y[1][0] == 1;
	
	
	// Remove if CASE-2
	//Consegna nella finestra [0-t], [FixedTime]
	ct7:
	sum(a in A1: a.j==1 && a.i!=1)
		z[a][t] == 1;
	
	
	forall(i in N, ti in T: ti>0) {
		ct8:
		sum(a in A1: i==a.j) f[a][ti-1]
		- sum(a in A1: i==a.i) f[a][ti] == y[i][ti];
	}
	
	forall(a in A1, ti in T) {
		ct9:
		f[a][ti] <= (t)*z[a][ti];
	}
	
	// Vincolo 1 arco per ogni istante
	forall(ti in T) {
		sum(a in A1)
		  z[a][ti] == 1;
		
	}
	// Vincolo per non ripassare su un arco in istanti diversi
	forall(a in A1: a.i!=a.j) {
		sum(ti in T)
		  z[a][ti] <= 1;	
	} 
	
	//----------------------------------//
	
	forall(di in D) {
		ct10:
		e[di][0] == Emax;		
	}
	
	forall(di in D, ti in T) {
		ct11:
		e[di][ti] <= Emax;
		ct12:
		e[di][ti] >= Emin;
	}
	

	forall(di in D, ti in T: ti>0) {
		ct18:
		e[di][ti] == (e[di][ti-1] - sum(a2 in A2) x[a2][di][ti] * a2.cost
					+ Emax*w[di][ti] - r[di][ti]);
	}
	
	forall(i in N, ti in T: ti>0) {
		ct14:
		sum(a in A1: a.i==i)
		  z[a][ti] == y[i][ti];
	}
	
	
	// New constraints for r
	forall(di in D, ti in T: ti>0) {
	  	ctr1:
		r[di][ti] <= e[di][ti-1];
		ctr2:
		r[di][ti] <= w[di][ti] * M;
		ctr3:
		r[di][ti] >= e[di][ti-1] - (1 - w[di][ti]) * M;
	}
	
	
	// New testing part 
	//forall(di in D, ti in T) {
	//	w[di][ti] + sum(a2 in A2) x[a2][di][ti] <= 1;  
	//}
	
	// New constraint for q
	forall(di in D, ti in T) {
		w[di][ti] + q[di][ti] <= 1;  
	}	
	
	// New constraint for q
	forall(a2 in A2, di in D, ti in T) {
		q[di][ti] >= x[a2][di][ti];  
	}
	
	// New constraint [time limit]
	forall(di in D, ti in T) {
		1/30 *sum(a2 in A2) x[a2][di][ti] * a2.dist  <= 0.5;
	}
	
	
	// Vincolo per fix: gli archi entranti sul deposito devo essere al pi� 1
	ct_fix:
	sum(ti in T, a in A1: a.i != 1 && a.j==1)
	  z[a][ti] <= 1;
	
	
	// New constraints for drone time slots
	forall(di in D, ti in T: ti>0) {
		I[di][ti] == TS - 1/30 * sum(a2 in A2) x[a2][di][ti] * a2.dist;
	}
	forall(di in D, ti in T: ti>0) {
		dmax[ti] >= I[di][ti];
	}	
	
	
	/*----------------------------------------------*/
	
	//nuovo vincolo
        forall(di in D, ti in T){
        sum(a2 in A2) x[a2][di][ti] <= (1- sum(a in A1: a.j!=a.i)z[a][ti]);
        }
    /*  
      
	//vincolo definizione variabile
		forall (di in D,ti in T){
        m1[di][ti]*M >= - (e[di][ti] - 0.5*(Emax-Emin));
        }


//vincolo che impone le ricariche
	forall (di in D,ti in T){
            w[di][ti] <= 1 + M*(1- sum(a in A1: a.j!=a.i) z[a][ti]) + M*(1-m1[di][ti]);
            w[di][ti] >= 1 - M*(1- sum(a in A1: a.j!=a.i) z[a][ti]) + M*(1-m1[di][ti]);
		}

//vincolo definizione variabile
	forall (di in D, ti in T){
	    m1[di][ti]*M >= (e[di][ti] - 0.5*(Emax-Emin));
   }	    

//vincolo relazione m1 e m2
	forall (di in D, ti in T){
    		m1[di][ti] + m2[di][ti] <=1;
    		}      
      */  
	
	res == sum(a in A1, ti in T:ti>0) a.cost * z[a][ti] + sum(a in A2, di in D, ti in T:ti>0) a.cost * x[a][di][ti]; //ti>0
	truck_km == sum(a in A1, ti in T) a.dist * z[a][ti];
	truck_en == sum(a in A1, ti in T) a.cost * z[a][ti];
	drone_km == sum(a in A2, di in D, ti in T) a.dist * x[a][di][ti];
	drone_en == sum(a in A2, di in D, ti in T) a.cost * x[a][di][ti];
	c_sat == sum(di in D, a in A2, ti in T: ti>0) x[a][di][ti];
	drone_energy_level == sum(di in D, ti in T) e[di][ti];
	obj_dmax == sum(ti in T: ti>0) (dmax[ti]);
	
	
}	

// Fix per le ricariche consecutive
int recharge[D][T]; // 1 se cliente raggiungibile 0 altrimenti
execute {
	for(var d in thisOplModel.D) {
	  var prec = 0;
	  for(var t in thisOplModel.T) {
	    if(prec == 1) {
	      if(w[d][t] == 0) {
	        prec = 0;
	      } else {
	      	recharge[d][t] = 0; 
	      }
	      continue;
	    }
	    if(w[d][t] == 1) {
	      recharge[d][t] = 1;
	      prec = 1;
	    }
	  }
	}
}


// Variante di visualizzazione del risultato
execute {
  
  var date2 = new Date();
  
  function format2 (f) {
	f = Math.round(f*100);
	var fs = f.toString();

	var s2 = fs.substring(0,2);
	if(s2.length == 1) fs = "0.0" + s2;
	else fs = "0."+ s2
	return fs;
  }
  
  var l = thisOplModel.lambda;
  var stringa ="";
  for(var term in terms) {
    stringa += format2(l[term]);
    if(term != thisOplModel.terms_num-1) stringa += "-";
  }
  
  var time = (date2.getTime()-thisOplModel.t_start)/1000;
  var gap = cplex.getMIPRelativeGap();
  if(gap < 1e-4) {
  	gap = 0; 
  }
  
  //-------------------Table construction---------------------//
  
  // EXCEL //
  var table = "log/table-"+thisOplModel.c+"C-"+thisOplModel.d+"D.csv";
  var arr= "";
  var f = new IloOplInputFile(table);
  while (!f.eof) {
  	var str=f.readline();
  	arr += str + ";";
  }
  f.close();
  var ar = arr.split(";");
  
  var f = new IloOplOutputFile(table);
  f.writeln(ar[0],",", lambda[0] + "-" + lambda[1] + "-" + lambda[2]); // Print the actual lambda combination
  //f.writeln(ar[0],",", thisOplModel.boat_en);
  f.writeln(ar[1],",", thisOplModel.truck_en);
  f.writeln(ar[2],",", thisOplModel.drone_en);
  //f.writeln(ar[3],",", thisOplModel.drone_energy_level);
  f.writeln(ar[3],",", "/");
  f.writeln(ar[4],",", lambda[0]*thisOplModel.truck_en/param[0]);
  f.writeln(ar[5],",", lambda[1]*thisOplModel.drone_en/param[1]);
  //f.writeln(ar[6],",", lambda[2]*thisOplModel.drone_energy_level/param[2]);
  f.writeln(ar[6],",", "/");
  f.writeln(ar[7],",", cplex.getObjValue());
  f.writeln(ar[8],",", gap);
  f.writeln(ar[9],",", time);
  f.close();
  
  //---------------------------------------------------------//
 
  var file = new IloOplOutputFile("log/output-"+"C"+thisOplModel.c+"-D"+thisOplModel.d+"_"+stringa+".txt");
  file.writeln("Clienti: "+thisOplModel.c+" - Droni: "+thisOplModel.d+" - TimeSlots: "+(thisOplModel.t+1))
  file.writeln("F.O. complessiva: "+ cplex.getObjValue());
  file.writeln("func_GAP: ", cplex.getMIPRelativeGap());
  file.writeln("calc_GAP: ", gap);
  //file.writeln("Valore ottimale funzione obiettivo: "+thisOplModel.res);
  file.writeln("Tempo di esecuzione in secondi: ", time);
  file.writeln("Consegne finite all'istante T: "+thisOplModel.tmax);
  
  file.writeln("\nNumero di termini FO: "+ thisOplModel.terms_num);
  file.writeln("Set di lambda:");
  for(var te in terms) {
  	file.writeln("lambda "+te+ " = " +l[te]); 
  }
  

  file.writeln("\nTermini in FO:");
  file.writeln("\tFO1: "+lambda[0]*thisOplModel.truck_en/param[0]);
  file.writeln("\tFO2: "+lambda[1]*thisOplModel.drone_en/param[1]);
  file.writeln("\tFO3: "+lambda[2]*thisOplModel.obj_dmax/7.5);
  
  
  var n = 0;
  for(var cl in C) {
  	if(reachable[cl] == 1) {
  		n++;  	
  	}  
  }
  
  file.writeln("\nTruck:");
  file.writeln("\tChilometri Percorsi: "+thisOplModel.truck_km+" km");
  file.writeln("\tEnergia utilizzata: "+thisOplModel.truck_en+" Wh");
  file.writeln("Drone/i:");
  file.writeln("\tChilometri Percorsi: "+thisOplModel.drone_km+" km");
  file.writeln("\tEnergia utilizzata: "+thisOplModel.drone_en+" Wh");
  file.writeln("\tTotale livello energia: "+thisOplModel.drone_energy_level+" Wh");
  file.writeln("\tClienti soddisfatti: "+thisOplModel.c_sat+"/"+n+" ("+thisOplModel.c+")");
  
  file.writeln("\tdmax sum: "+ thisOplModel.obj_dmax);
  
    file.writeln("\nClienti serviti dalle tappe (%,n/C):")
  for(var n in thisOplModel.N) {
    var cnt = 0;
    for(var t in thisOplModel.T){
	  	for(var a in thisOplModel.A2) {
			for(var d in thisOplModel.D) {
				if(a.i == n && x[a][d][t] == 1) {
					cnt += 1;  			
				}  				  		
	  		}  	
	  	} 	
	 }
  	file.writeln("\t"+"Tappa "+n+": "+100*(cnt/thisOplModel.c)+"% - "+cnt+"/"+thisOplModel.c);
  }
  
  file.writeln("\nConsegne effettuate con i droni:")
  for(var t in thisOplModel.T){
  	for(var a in thisOplModel.A2){
		for(var d in thisOplModel.D){
			if(x[a][d][t] == 1 && t!=0) {
				file.writeln("\t"+"Nodo: "+a.i+" Cliente: "+a.j+" Drone: "+d+" Time: "+t);  			
			}  				  		
  		}  	
  	 } 	
  }
  
  file.writeln("\nArchi percorsi dal truck:");
  for(var t in thisOplModel.T){
  	for(var a in thisOplModel.A1){
	  	if(z[a][t] == 1) {
	    	file.writeln("\t"+"Nodo1: "+a.i+" Nodo2: "+a.j+" Time: "+t);
	  	}
 	}  
  }
  
  file.writeln("\nRicariche dei droni:");
  for(var d in thisOplModel.D){
  	for(var t in thisOplModel.T){
	  	if(recharge[d][t] == 1) {
	    	file.writeln("\t"+"Drone: "+d+" Time: "+t);
	  	}
 	}
 	file.writeln("");  
  }
  
  file.writeln("\nLivello di energia dei droni:");
  for(var d in thisOplModel.D){
  	for(var t in thisOplModel.T){
	    file.writeln("\t"+"Drone: "+d+" Time: "+t+" = "+e[d][t]);
 	}
 	file.writeln("");  
  }
  
   // Idle time calculation
  file.writeln("\nTempo di inattivit� (idle time) in minuti:");
  var idle_time = 0
  for(var t in thisOplModel.T){
    var max_time = 0;
    for(var d in thisOplModel.D){
      var tot_dist = 0;
      for(var a in thisOplModel.A2){       
      	tot_dist += x[a][d][t] * a.dist
      	//dlv_time += 1/30 * (x[a][d][t] * a.dist);
      }
      var dlv_time = 1/30 * tot_dist
      if(dlv_time > max_time) {
      	  max_time = dlv_time;
      }
   	}
   	file.writeln("T:", t, " - Idle time: ", (0.5 - max_time)*60);
   	if(max_time != 0) idle_time += (0.5 - max_time)*60;   	
  }
  file.writeln(""); 
  file.writeln("Total idle time: ", idle_time);  
  
 file.close();
}


// GraphMaker //
/*
execute {
  function customerDone(cd, ti) {
 	for(var i=ti; i<thisOplModel.t + 1; i++) {
 		file.writeln("c"+cd+"_"+i+" [fillcolor=green];");
 	}  	
  }
  var file = new IloOplOutputFile("output.txt");

  // Edges of the tour
  file.writeln("// Truck's edges:");
  for(var a in thisOplModel.A1){
	for(var t in thisOplModel.T){
	  	if(z[a][t] == 1 && t < thisOplModel.t) {
	    	file.writeln("n"+a.i+"_"+t+" -> "+"n"+a.j+"_"+(t+1)+";");
	  	}
 	}  
  }
  
  // Edges of the drones
  file.writeln("// Drones' edges:");
  for(var a in thisOplModel.A2) {
  	for(var d in thisOplModel.D) {    
	  for(var t in thisOplModel.T) {
	  	if(x[a][d][t] == 1) {
	  		// Roundtrip
	  		var attributes = " [style=dashed, penwidth=2, constraint=false, color="+thisOplModel.colors[d]+"];";
	  		file.writeln("n"+a.i+"_"+t+" -> "+"c"+a.j+"_"+t+attributes);
	    	file.writeln("c"+a.j+"_"+t+" -> "+"n"+a.i+"_"+t+attributes);    	
	    	customerDone(a.j, t);	
	  	}
  	  }	  	
 	}  
  }
  
  // Nodes visited
  file.writeln("// Nodes visited:");
  for(var n in thisOplModel.N){
	for(var t in thisOplModel.T){
	  	if(y[n][t] == 1) {
	    	file.writeln("n"+n+"_"+t+" [fillcolor=red];");
	  	}
 	}  
  }
 file.close();
}
*/

// Clienti effettivamente raggiungibili
execute {
  var file = new IloOplOutputFile("customers.txt");
  var j = 1;
  file.writeln("clienti = {");
  for(var cl in thisOplModel.clienti) {
  	if(reachable[j] == 1) {
  	  file.writeln("<"+cl.lat+", "+cl.lon+">,");		
	}
	j++;		
  }	
 file.writeln("};");
 file.close();
}


main {
    var opl = thisOplModel
    var mod = opl.modelDefinition;
    var dat = opl.dataElements;
	var file_name = "Roma_Scenario";
	
	var l = dat.lambda;
	var terms_num = opl.terms_num;
	var div = 1 / opl.terms_num; 
    var step = thisOplModel.lambda_step;
    writeln("Div is : ", div);
    writeln("Step is : ", step);
    
    //-------------------Reset of table----------------------//
    var f = new IloOplOutputFile("log/table-"+thisOplModel.c+"C-"+thisOplModel.d+"D.csv");
    for(var num = 0; num < thisOplModel.table_terms; num++) {
    	f.writeln("");
  	}
  	f.close();
  	//------------------------------------------------------//
    
    for(var k in thisOplModel.terms) l[k] = div; 
    
    //-------------------Equal lambdas case---------------------//
    var cplex1 = new IloCplex();
    opl = new IloOplModel(mod,cplex1);
    opl.addDataSource(dat);
    opl.generate();
    if (!cplex1.solve()) {
        writeln("ERROR: could not solve");
        opl.end();
    } else {
    	opl.postProcess();
  	}    
    writeln("Current solution : ", cplex1.getObjValue());
    //---------------------------------------------------------//
    /*
    for(var i in thisOplModel.terms) {
      	l[i] = div * step;
      	for(var j in thisOplModel.terms) {
      	  	if(i==j) continue;
		 	var cplex1 = new IloCplex();
		 	l[j] = div + div - (div * step);
		 	
	        opl = new IloOplModel(mod,cplex1);
	        opl.addDataSource(dat);
	        opl.generate();
	        if (!cplex1.solve()) {
	            writeln("ERROR: could not solve");
	            status = 1;
	            opl.end();
	            break;
	        }
	        opl.postProcess();
	        writeln("Current solution : ", cplex1.getObjValue());
	
	        
			l[j] = div;
  		}
  		l[i] = div;
    }
    */
    //opl.end();
	cplex1.end();   
}  

