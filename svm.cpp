#include<iostream>
#include<math.h>

#define CNT 8
#define MAX_VAL 99999L
#define MIN_GAP 0.00001

using namespace std;

class Point{
	double x1;
	double x2;
public:
	Point() {};
	
	Point(double _x1, double _x2) : x1(_x1), x2(_x2) {};
	
	// compare use in sorting algorithm to make a convex hull.
	int compare(Point p1, Point p2){
		int ret_val;
		double calc_val;
		
		// calulate differente of slope ( between this and p1 ) and ( between this and p2 )
		// (p1.x2 - x2) / (p1.x1 - x1) > (p2.x2 - x2) / (p2.0x1 - x1) == p1 > p2
		calc_val = (p1.x2-x2) * (p2.x1 - x1) - (p2.x2 - x2) * (p1.x1 - x1);
		// positive value
		if( calc_val > MIN_GAP ){
			ret_val = 1;
		}
		// negative value
		else if( calc_val < -MIN_GAP ){
			ret_val = -1;
		}
		// equals
		else{
			ret_val = 0;
		}
		
		return ret_val;
	}
	
	// get signed area. convex hull maded by positive direction
	double signedArea(Point p1, Point p2){
		// det value
		return x1*p1.x2 + p1.x1*p2.x2 + p2.x1*x2
			 - (x2*p1.x1 + p1.x2*p2.x1 + p2.x2*x1);
	}
	
	void printPoint(string str){
		cout << str << "{" << x1 << ", " << x2 << "}," << endl;
	}
	
	// move value
	void operator =(Point p1){
		x1 = p1.x1;
		x2 = p1.x2;
	}
	
	// get distance between this and p
	double getDistance(Point p){
		return sqrt( (x1-p.x1)*(x1-p.x1) + (x2-p.x2)*(x2-p.x2) );
	}
	
	// get distance between eaulation w and point(this)
	double getWeightDistance(double w[3]){
		// abs( w1*x1 + w2*x2 + w3) / sqrt( w[0]*w[0] + w[1]*w[1] ) 
		return w[0]*x1 + w[1]*x2 + w[2] / sqrt( w[0]*w[0] + w[1]*w[1] );
	}
	
	// set weight value on w using ( metholine vector made by this and p ), and ( middle point between this and p )
	void getWeight(Point p, double w[3]){
		// methodline vector
		w[0] = x1-p.x1;
		w[1] = x2-p.x2;
		
		// w1*x1 + w2*x2 + w3 = 0; w1*(x1-mid_x1) + w2*(x2-midx2) = 0;
		double mid_x1 = (x1 + p.x1) / 2;
		double mid_x2 = (x2 + p.x2) / 2;
		
		w[2] = -( w[0] * mid_x1 + w[1] * mid_x2 );
	}
	
	// set weight value on w using ( metholine vector made by p1 and p2 ), and ( middle point between this and p1 )
	void getWeight(Point p1, Point p2, double w[3]){
		// methodline vector using linear equation between p1 and p2
		w[0] = p1.x2-p2.x2;
		w[1] = -(p1.x1-p2.x1);
		
		// w1*x1 + w2*x2 + w3 = 0; w1*(x1-mid_x1) + w2*(x2-midx2) = 0;
		double mid_x1 = (x1 + p1.x1) / 2;
		double mid_x2 = (x2 + p1.x2) / 2;
		
		w[2] = -( w[0] * mid_x1 + w[1] * mid_x2 );
	}
	
	friend class ConvexHull;
};

class ConvexHull{
	Point points[CNT+1];
	int length;
public:
	ConvexHull() : length(0) {};
	
	int getLength(){
		return length;
	}
	
	Point getPoint(int index){
		return points[index];
	}
	
	void addPoint(double x1, double x2){
		points[length] = Point(x1, x2);
		length += 1;
	}
	
	void addPoint(Point p){
		points[length] = Point(p.x1, p.x2);
		length += 1;
	}
	
	void popPoint(){
		swapPoint(length-2, length-1);
		length -= 1;
	}
	
	void swapPoint(int index1, int index2){
		Point p;
		p = points[index1];
		points[index1] = points[index2];
		points[index2] = p;
	}
	
	void copyConvex(ConvexHull convex){
		int i;
		int len;
		
		len = convex.length;
		length = 0;
		for(i=0; i<len; i++){
			addPoint( convex.getPoint(i) );
		}
	}
	
	// sort poinst using compare function in Point
	void sortPoints();
	
	// make a convex hull using Graham's scan
	void makeConvexHull();
	
	void printState(){
		int i;
		cout << "{" << endl;
		for(i=0;i<length; i++){
			points[i].printPoint("  ");
		}
		cout << "}" << endl;
	}
	
	friend class SVM;
};

// sort poinst using compare function in Point
void ConvexHull::sortPoints(){
	int i, j;
	bool flag;
	int index = 0;
	Point base_point(points[0]);
	
	// calculate start value, minimum x1
	for(i=1; i<length; i++){
		if( base_point.x2 > points[i].x2 ){
			index = i;
			base_point = points[i];
		}
		else if( base_point.x2 == points[i].x2 && base_point.x1 > points[i].x1 ){
			index = i;
			base_point = points[i];
		}
	}
	
	// set the fist point
	swapPoint(0, index);
	
	// bubble sort
	for(i=0; i<length-1; i++){
		flag = true;
		for(j=1; j<length-1-i; j++){
			if( points[0].compare(points[j], points[j+1]) > 0 ){
				swapPoint(j, j+1);
				flag = false;
			}
		}
		if( flag ){
			break;
		}
	}
}

// make a convex hull using Graham's scan
void ConvexHull::makeConvexHull(){
	ConvexHull ret_convex;
	Point last_point;
	Point second_last_point;
	Point third_last_point;
	int i;
	int ret_len;
	double signed_val;
	
	// sort points
	sortPoints();
	
	// make convexHull; don't need to edit init two value.
	ret_convex.addPoint(points[0]);
	ret_convex.addPoint(points[1]);
	for(i=2; i<length; i++){
		// add point
		ret_convex.addPoint(points[i]);
		do {
			ret_len = ret_convex.getLength();
			
			// get signed area
			last_point = ret_convex.getPoint(ret_len-1);
			second_last_point = ret_convex.getPoint(ret_len-2);
			third_last_point = ret_convex.getPoint(ret_len-3);
			signed_val = third_last_point.signedArea(second_last_point,last_point);
			
			// if negative value than remove last point
			if( signed_val <= 0 ){
				ret_convex.popPoint();
				
				ret_len -= 1;
			}
			
		} while( ret_len > 2 && signed_val <= 0 );
	}
	// add start point to last point
	ret_convex.addPoint(points[0]);
	do {
		ret_len = ret_convex.getLength();
		
		// get signed area
		last_point = ret_convex.getPoint(ret_len-1);
		second_last_point = ret_convex.getPoint(ret_len-2);
		third_last_point = ret_convex.getPoint(ret_len-3);
		signed_val = third_last_point.signedArea(second_last_point,last_point);
		
		// if negative value than remove last point
		if( signed_val <= 0 ){
			ret_convex.popPoint();
			
			ret_len -= 1;
		}
		
	} while( ret_len > 2 && signed_val <= 0 );
	
	// set this value
	copyConvex(ret_convex);
	
}

class SVM{
	ConvexHull positive_convex;
	ConvexHull negative_convex;
	double w[3];
	double max_distance;
public:
	SVM(double _x[CNT][2], double _y[CNT]){
		int i;
		for(i=0; i<CNT; i++){
			if( _y[i] == 1 ){
				positive_convex.addPoint(_x[i][0], _x[i][1]);
			}
			else{
				negative_convex.addPoint(_x[i][0], _x[i][1]);
			}
		}
		positive_convex.makeConvexHull();
		negative_convex.makeConvexHull();
	}
	
	// calulate weight value from possible points and edges
	void learning();
	
	void printState(){
		cout << "positive_convex : " << endl;
		positive_convex.printState();
		cout << "negative_convex : " << endl;
		negative_convex.printState();
	}
	
};

/**
 * this is the calculate weight value from possible poinsts and edges
 * 1. calculate posible points and edges
 * 2. get a maximum distance from possible edges.
 * 3. get a maximum distance from possible points.
 * 4. print weight and maiximum distance.
 */
void SVM::learning(){
	int positive_len = positive_convex.length;
	int negative_len = negative_convex.length;
	bool positive_target_point[CNT+1];
	bool negative_target_point[CNT+1];
	Point base_point1;
	Point base_point2;
	double signed_area_val;
	double max_signed_area_val;   // it is negative value
	int max_signed_area_state[3]; // (positive or negative), target_index, min_index_in_oppersite
	int i, j, k;
	int len;
	bool flag;
 	double distance, tmp_distance;
	bool is_valid;
	double tmp_w[3];
	
	// init value
	for( i=0; i<CNT; i++ ){
		positive_target_point[i] = false;
		negative_target_point[i] = false;
	}
	
	// get all positive target edges and calculate max distance
	max_signed_area_val = -MAX_VAL;
	len = positive_len-1;
	for( i=0; i<len; i++ ){
		flag = true;
		base_point1 = positive_convex.getPoint(i);
		base_point2 = positive_convex.getPoint(i+1);
		for( j=0; j<negative_len-1; j++ ){
			// if exists positive value then it is not taget point
			signed_area_val = base_point1.signedArea(base_point2, negative_convex.getPoint(j));
			if( signed_area_val >= 0 ){
				flag = false;
				break;
			}
			else{
				if( max_signed_area_val < signed_area_val ){
					max_signed_area_val = signed_area_val;
					max_signed_area_state[0] = 1;
					max_signed_area_state[1] = i;
					max_signed_area_state[2] = j;
				}
			}
		}
		if( flag == true ){
			positive_target_point[i] = true;
			positive_target_point[i+1] = true;
		}
	}
	// get all negative target edge and calculate max distance
	len = negative_len-1;
	for( i=0; i<len; i++ ){
		flag = true;
		base_point1 = negative_convex.getPoint(i);
		base_point2 = negative_convex.getPoint(i+1);
		for( j=0; j<positive_len-1; j++ ){
			// if exists positive value then it is not taget point
			signed_area_val = base_point1.signedArea(base_point2, positive_convex.getPoint(j));
			if( signed_area_val >= 0 ){
				flag = false;
				break;
			}
			else{
				if( max_signed_area_val < signed_area_val ){
					max_signed_area_val = signed_area_val;
					max_signed_area_state[0] = 0;
					max_signed_area_state[1] = i;
					max_signed_area_state[2] = j;
				}
			}
		}
		if( flag == true ){
			negative_target_point[i] = true;
			negative_target_point[i+1] = true;
		}
	}
	
	// get weight value and distance
	// positive than
	if( max_signed_area_state[0] == 1 ){
		// oppersite point
		base_point1 = negative_convex.getPoint(max_signed_area_state[2]);
		base_point1.getWeight(
			positive_convex.getPoint(max_signed_area_state[1]),   // start point
			positive_convex.getPoint(max_signed_area_state[1]+1), // end point
			w);
	}
	else{
		// oppersite point
		base_point1 = positive_convex.getPoint(max_signed_area_state[2]);
		base_point1.getWeight(
			negative_convex.getPoint(max_signed_area_state[1]),   // start point
			negative_convex.getPoint(max_signed_area_state[1]+1), // end point
			w);
	}
	max_distance = base_point1.getWeightDistance(w);
	
	// calculate distance in target node 
	for( i=0; i<CNT; i++ ){
		for( j=0; j<CNT; j++ ){
			if( positive_target_point[i] == true && negative_target_point[j] == true ){
				is_valid = true;
				
				base_point1 = positive_convex.getPoint(i);
				base_point2 = negative_convex.getPoint(j);
				
				base_point1.getWeight(base_point2, tmp_w);
				
				distance = base_point1.getDistance(base_point2) / 2;
				
				for( k=0; k<CNT; k++ ){
					if( positive_target_point[k] == true && k != i ){
						tmp_distance = positive_convex.getPoint(k).getWeightDistance(tmp_w);
						
						if( distance - tmp_distance > MIN_GAP ){
							is_valid = false;
							break;
						}
					}
				}
				for( k=0; k<CNT; k++ ){
					if( positive_target_point[k] == true && k != j ){
						tmp_distance = negative_convex.getPoint(k).getWeightDistance(tmp_w);
						
						if( distance - tmp_distance > MIN_GAP ){
							is_valid = false;
							break;
						}
					}
				}
				
				if( is_valid == true ){
					if( distance > max_distance ){
						for( k=0; k<3; k++ ){
							w[k] = tmp_w[k];
						}
						max_distance = distance;
					}
				}
			}
		}
	}
	
	cout << "distance : " << max_distance << endl;
	cout << "weight : " << w[0] << ", " << w[1] << ", " << w[2] << endl;
	
}

int main(){
	// input data
	double x[CNT][2] = { 
		  {-1, 3}
		, {1, 2}
		, {1, 3}
		, {0, 0}
		, {2, 0}
		, {4, 0}
		, {2, 1}
		, {3, 2}
	};
	// output data
	double y[CNT] = {
		  1
		, 1
		, 1
		, 1
		, 0
		, 0
		, 0
		, 0
	};
	
	SVM svm(x, y);
	
	// print two differenct field
	svm.printState();
	
	// calculate a weightValue. and print weight and maximum distance
	svm.learning();
	
    return 0;  
}
