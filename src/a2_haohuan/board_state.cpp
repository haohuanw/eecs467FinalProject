#include "board_state.hpp"
using namespace std;

BoardState::BoardState() {
  ifstream cyan_file("../calibration/cyan_centers.txt");
  eecs467::Point<int> c;
  while(cyan_file >> c.x >> c.y){
    cyan_squares.push_back(c);
    cout << c.x << ' ' << c.y << endl;
  }
  board_squares = vector<eecs467::Point<double>>(9);
  cout << '\t' << board_squares.size() << endl;
  
  
}

int BoardState::ballsLeft() {
  return availBalls.size();
}

eecs467::Point <double> BoardState::nextFreeBall() {
  eecs467::Point <double> point = availBalls[0];
		
  availBalls.erase(availBalls.begin());
  std::cout << '\t' << point.x << ' ' << point.y << std::endl;
  return point;
}
	
std::vector <char> BoardState::determineStateofBoard(std::vector <int>& greenBalls, 
						     std::vector <int>& redBalls, int imgWidth, int imgHeight,calibration_t &cali, char color) {
	
  //clear vectors
  availBalls.clear();
	

  //initilize board
  std::vector<char> camBoard;
  for(unsigned int i = 0; i < 9; i++) 
    camBoard.push_back(' ');

  //get distance between registration squares (pixels to meters)
  //not needed annymore, will keep temporarily for sanity checks
  eecs467::Point<double> sq1Point = cyan_squares[0];
  eecs467::Point<double> sq4Point= cyan_squares.back();
   std::cout << "    cyan " << sq1Point.x << "," << sq1Point.y << std::endl;
  std::cout << "    cyan " << sq4Point.x << "," << sq4Point.y << std::endl;
  /*
  std::sort(cyanSquares.begin(), cyanSquares.end());

  std::cout<<"sorted cyanSquares here"<<std::endl;
  if(cyanSquares.size() != 4)
    std::cout << "Too many or too few cyan squares" << endl;

  sq1Point.x = cyanSquares[0]%imgWidth;
  sq1Point.y = imgHeight - (cyanSquares[0]/imgWidth);

  sq4Point.x = cyanSquares[3]%imgWidth;
  sq4Point.y = imgHeight - (cyanSquares[3]/imgWidth);
  std::cout<< "sq1Point: "<<sq1Point.x<<" "<< sq1Point.y<<std::endl;
  std::cout<< "sq2Point: "<<sq4Point.x<<" "<< sq4Point.y<<std::endl;
  */
  sq1Point.y = imgHeight - sq1Point.y;
  sq4Point.y = imgHeight - sq4Point.y;
  sq1Point = cali.translate(sq1Point);
  sq4Point = cali.translate(sq4Point);

  int q = convert(sq1Point, sq1Point, sq4Point);

  std::cout << "cyan " << sq1Point.x << "," << sq1Point.y << std::endl;
  std::cout << "cyan " << sq4Point.x << "," << sq4Point.y << std::endl;
  //exit(1);
  //parameters used to calulate board position
  double x_o = sq1Point.x;
  double y_o = sq1Point.y;
  double distBetSquares = sqrt(sq(sq4Point.y - sq1Point.y)+sq(sq4Point.x-sq1Point.x));
  std::cout <<"distBetSquares:"<< distBetSquares << std::endl;

  //getting board coordinates for the balls
  double red_max, green_max;
  /*if(color == 'R'){
    red_max = 0;
    green_max = sq4Point.x - sq1Point.x;
    cout << "Green max: " << green_max << endl;
    }
    else{
    red_max = sq4Point.x - sq1Point.x;
    green_max = 0;
    cout << "Red max: " << red_max << endl;
    }*/
  eecs467::Point <double> point;
  if(color == 'R'){
    for (unsigned int i = 0; i < redBalls.size(); i++) {

      point.x = redBalls[i]%imgWidth; 
      point.y = imgHeight - (redBalls[i]/imgWidth); 
			
      point = cali.translate(point);


      std::cout << "redBall \t" << point.x << "," << point.y << std::endl;
      int index;
      if( (point.x > 0) || (point.y >= fabs(sq1Point.y)) || (point.y <= -1*fabs(sq1Point.y)) ){
	if(color == 'R') {
	  std::cout << "Pushing back: " << point.x << ' ' << point.y << std::endl;
	  availBalls.push_back(point);
	}
      }
      else {
	index = convert(point, sq1Point, sq4Point); //converts to board coordinates
	if(index != -1)
	  camBoard[index] = 'R';
	std::cout << "Red Pos: " << index << std::endl;
      }

    }
		

    for (unsigned int i = 0; i < greenBalls.size(); i++) {
			
      point.x = greenBalls[i]%imgWidth; 
      point.y = imgHeight - (greenBalls[i]/imgWidth); 
			
      point = cali.translate(point);

      std::cout << "greenBall \t" << point.x << "," << point.y << std::endl;
      int index;
      if( (point.x < (sq4Point.x - sq1Point.x)) || (point.y >= fabs(sq1Point.y)) || (point.y <= -1*fabs(sq1Point.y))){
	if(color == 'G')
	  availBalls.push_back(point);
      }

      else {
	index = convert(point, sq1Point, sq4Point); //converts to board coordinates
	if(index != -1)
	  camBoard[index] = 'G';
	std::cout << "Green Pos: " << index << std::endl;
      }

    }
  }
  if(color == 'G'){
    for (unsigned int i = 0; i < redBalls.size(); i++) {

      point.x = redBalls[i]%imgWidth; 
      point.y = imgHeight - (redBalls[i]/imgWidth); 
			
      point = cali.translate(point);


      std::cout << "redBall \t" << point.x << "," << point.y << std::endl;
      int index;
      if( (point.x < (sq4Point.x - sq1Point.x)) || (point.y >= fabs(sq1Point.y)) || (point.y <= -1*fabs(sq1Point.y)) ){
	if(color == 'R') {
	  std::cout << "Pushing back: " << point.x << ' ' << point.y << std::endl;
	  availBalls.push_back(point);
	}
      }
      else {
	index = convert(point, sq1Point, sq4Point); //converts to board coordinates
	if(index != -1)
	  camBoard[index] = 'R';
	std::cout << "Red Pos: " << index << std::endl;
      }

    }
		

    for (unsigned int i = 0; i < greenBalls.size(); i++) {
			
      point.x = greenBalls[i]%imgWidth; 
      point.y = imgHeight - (greenBalls[i]/imgWidth); 
			
      point = cali.translate(point);

      std::cout << "greenBall \t" << point.x << "," << point.y << std::endl;
      int index;
      if( (point.x > 0) || (point.y >= fabs(sq1Point.y)) || (point.y <= -1*fabs(sq1Point.y))){
	if(color == 'G')
	  availBalls.push_back(point);
      }

      else {
	index = convert(point, sq1Point, sq4Point); //converts to board coordinates
	if(index != -1)
	  camBoard[index] = 'G';
	std::cout << "Green Pos: " << index << std::endl;
      }

    }
  }	

  return camBoard;			
}

int BoardState::convert(eecs467::Point<double> p, eecs467::Point<double> sq1, eecs467::Point<double> sq4){

  //double len = sqrt(sq(sq1.x + sq4.x) + sq(sq1.y + sq4.y));
  double sqSize = gridCellSize;
  eecs467::Point<double> center;
  center.x = (sq1.x + sq4.x)/2.0;
  center.y = (sq1.y + sq4.y)/2.0;
  
  //vector<eecs467::Point<double>> b(9);

  /*
   *  b0   b1   b2
   *  b3   b4   b5
   *  b6   b7   b8
   */
  board_squares[0].x = center.x + sqSize;
  board_squares[0].y = center.y + sqSize;

  board_squares[1].x = center.x + sqSize;
  board_squares[1].y = center.y;

  board_squares[2].x = center.x + sqSize;
  board_squares[2].y = center.y - sqSize;

  board_squares[3].x = center.x;
  board_squares[3].y = center.y + sqSize;

  board_squares[4].x = center.x;
  board_squares[4].y = center.y;
  
  board_squares[5].x = center.x;
  board_squares[5].y = center.y - sqSize;

  board_squares[6].x = center.x - sqSize;
  board_squares[6].y = center.y + sqSize;

  board_squares[7].x = center.x - sqSize;
  board_squares[7].y = center.y;
  
  board_squares[8].x = center.x - sqSize;
  board_squares[8].y = center.y - sqSize;


  double minDist = 1.0;
  int closest = -1;
  for(int i=0; i<board_squares.size(); ++i){
    double pD = sqrt(sq(p.x - board_squares[i].x) + sq(p.y - board_squares[i].y));
    if(pD < minDist){
      minDist = pD;
      closest = i;
    }
  }
  
  
  return closest;
}

