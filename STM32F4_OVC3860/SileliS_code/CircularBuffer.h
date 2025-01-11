/**
  ******************************************************************************
  * @file    CircularBuffer.h
  * @author  Dawid "SileliS" Bańkowski
  * @brief   Circular Buffer class.
  *          This file provides code to manage circular buffer.
  *          It is platform independend.
  *          based on: //https://github.com/embeddedartistry/embedded-resources
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Dawid Bańkowski.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by GNU Public Licence, * the "License" *;
  * You may not use this file except in compliance with the License and name of author.
  *
  ******************************************************************************
  */
#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_


#include <cstdio>
#include <cstring>

//TODO: translate comments to english (I did not do this because lack of time)

using namespace std;

/**
  * @brief  circularBufferSearchResult is helpfull to return sopfisticated search results.
  *
  * @param  n/a
  *
  * @param  n/a
  *         n/a
  * @retval isFound - bool variable which value is TRUE (information had been found in buffer)
  * 		tail2virtualTail_ - liczba (size_t), która informuje o odległości pomiędzy tail_,
  * 		 a pozycją, w której odnaleziono informacje. W szczególnych przypadkach może
  * 		 przyjmować 0, gdy:
  * 				- isFound = flase (nie znaleziono informacji)
  * 				- szukana informacja znajduje sie w pozycji tail_
  */
typedef struct circularBufferSearchResult{
	bool isFound;
	size_t tail2virtualTail_;
};

/**
* @brief  	CircularBuffer to klasa obsługująca bufor kołowy.
* @param  	T 		- typ zmiennej jaka ma być przechowywana w buforze kołowym
* 			Size	- wielkośc bufora kołowego
*
*/
template<typename T, size_t Size> class CircularBuffer {

public:
	CircularBuffer(void);
	~CircularBuffer(void);
	void put(T item);
	T get(void);
	bool isEmpty(void) const;
	bool isFull(void) const;
	bool isOverflowed(void) const;
	void resetCircularBuffer(void);
	size_t capacity(void) const;
	size_t dataSize(void) const;
	bool searchItemTail2Range(const void *item, size_t items, size_t range, bool tailPosUpdate= true);
	circularBufferSearchResult SearchItemTail2Range(const void *item, size_t items, size_t range, bool tailPosUpdate= true);
	bool searchItem(const void* item, size_t items, bool tailPosUpdate = true);
	circularBufferSearchResult SearchItem(const void* item, size_t items, bool tailPosUpdate = true);



protected:

private:
	T buf_[Size];									//bufor
	size_t head_ = 0;								//miejsce zapisu danych
	size_t tail_ = 0;								//miejsce odczytu danych
	size_t max_size_ = Size;						//pojemnośc bufora
	bool overflow = false;							//informacja o przepełnieniu bufora
	bool full_ = 0;									//zmienia wartośc na 1 jeśli bufor zawiera conajmniej 1 daną
	void clearBuffer(void);							//czyści bufor
};



/****************************************************************************************************
*
*
*
*
****************************************************************************************************/


/**
* @brief  Konstruktor klasy.
* @note   example: CircularBuffer<char, 25> nameOfBuffer;
* @param  n/a.
* @note   n/a.
* @retval n/a.
*/
template<typename T, size_t Size> CircularBuffer<T, Size>::CircularBuffer(void) {
	clearBuffer();
}


/**
* @brief  destruktor klasy.
* @note   n/a.
* @param  n/a.
* @note   Czyści dane w obiekcie.
* @retval n/a.
*/
template<typename T, size_t Size> CircularBuffer<T, Size>::~CircularBuffer(void) {
	head_ = 0;
	tail_ = 0;
	max_size_ = Size;
}



/**
* @brief  Zeruje parametry obiektu i czyści jego zawartość.
* @note   Wymagane m.in. w przypadku, gdby bufor został przepełniony.
*
* @param  n/a.
*
* @note   Zmienia parmetry: head_, full_ , overflow
*
* @retval Zwraca: n/a.
*/
template <typename T, size_t Size> void CircularBuffer<T, Size>::resetCircularBuffer(void) {
	head_ = tail_;
	full_ = false;
	overflow = false;
	clearBuffer();
}

template <typename T, size_t Size> void CircularBuffer<T, Size>::clearBuffer(void) {
	memset(buf_, NULL, sizeof(T)*Size);
}




/**
* @brief  Sprawdza czy w buforze znajdują się jakieś nieprzeczytane dane.
* @note   n/a.
*
* @param  n/a.
*
* @note   n/a.
*
* @retval Zwraca:
* 			- TRUE jeśli bufor jest pusty
* 			- FALSE jeśli w buforze znajduje się jakakolwiek nieprzeczytana informacja
*/
template<typename T, size_t Size> bool  CircularBuffer<T, Size>::isEmpty(void) const {
	return (!full_ && (head_ == tail_));
}


/**
* @brief  Sprawdza czy do bufora można wpisać jeszcze jakieś dane.
* @note   n/a.
*
* @param  n/a.
*
* @note   n/a.
*
* @retval Zwraca:
*  			- TRUE jeśli bufor jest pełny dataSize == max_size
*  			- FALSE jeśli w buforze jest jeszcze miejsce do zapisania danych
*/
template<typename T, size_t Size> bool  CircularBuffer<T, Size>::isFull(void) const {
	return full_;
}


/**
* @brief  Zwraca informację czy bufor nie został przepełniony.
* @note   Przepełnienie oznacza brak spójności w konsystencji danych
*
* @param  n/a.
*
* @note   n/a.
*
* @retval Zwraca: TRUE FALSE
*/
template<typename T, size_t Size> bool CircularBuffer<T, Size>::isOverflowed(void) const {
	return overflow;
}


/**
* @brief  Zwraca informację na temat max pojemności bufora.
* @note   n/a.
*
* @param  n/a.
*
* @note   n/a.
*
* @retval Zwraca wartośc (size_t) na temat maksymalniej pojemnosci bufora
*/
template<typename T, size_t Size> size_t CircularBuffer<T, Size>::capacity(void) const {
	return max_size_;
}



/**
* @brief  Zwraca informację na temat ilości danych przechowywanych w buforze.
* @note   n/a.
*
* @param  n/a.
*
* @note   n/a.
*
* @retval Zwraca wartośc (size_t) ilości danych przechowywanych w buforze
*/
template<typename T, size_t Size> size_t CircularBuffer<T, Size>::dataSize(void) const {

	size_t size = max_size_;
	if (!full_) {
		if (head_ >= tail_) {
			return size = head_ - tail_;
		} else {
			return size = max_size_ + head_ - tail_;
		}
	}

	return size;
}


/**
* @brief  Wstawia do obiektu daną.
* @note   Wstawia w miejsce wskazane przez head_

* @param  n/a.
* @note   Uaktualnia przy tym następujące zmienne w klasie:
* 		- head_
* 		- full,
* 		- overflow
* @retval n/a.
*/
template<typename T, size_t Size> void CircularBuffer<T, Size>::put(T item) {				//put item into buffer
	buf_[head_] = item;

	if (full_) {
		tail_ = (tail_ + 1) % max_size_;
		overflow = true;
	}

	head_ = (head_ + 1) % max_size_;
	full_ = head_ == tail_;
}



/**
* @brief  Czyta dane z bufora kołowego.
* @note   Czyta daną z bufora kołowego wskazaną przez tail_
*
* @param  n/a.
*
* @note   Uaktualnia przy tym następujące zmienne w klasie:
* 		- full,
* 		- tail_
*
* @retval Zwraca:
* 			- warość typu T znajdującą się w przeczytanej komórce bufora jeśli isEmpty()!= TRUE
* 			- T() jeśli isEmpty()== TRUE
*/
template<typename T, size_t Size> T CircularBuffer<T, Size>::get(void) {					//read item from buffer
	if (isEmpty()) {
		return T();
	}
	T val = buf_[tail_];
	full_ = false;
	tail_ = (tail_ + 1) % max_size_;

	return val;
}


/**
* @brief  Sprawdza czy bufor kołowy zawiera interesujące nas dane.
* @note   Wyszukuje w zakresie od tail_ do tail_ + range
*
* @param  *item 		- wskaźnik do bufora zawierającego szukane informacje
* 		  items 		- ilość danych typu T jaka ma być wyszukana
* 		  tailPosUpdate - informuje obiekt czy po znalezieniu szykanej wartości ustawić na nią tail_
* 		  					TRUE - wartośc domyśla oznacza, że tail_ powinien być uaktualniony
* 		  					FALSE - oznacza, że tail_ nie powinien być uaktualniony
* @note   n/a.
*
* @retval Zwraca strukturę circularBufferSearchResult o następujących polach:
		  - isFound przyjmujących wartości:
* 				- TRUE jeśli szukana wartośc została odnaleziona
* 				- FALSE jeśli: 	szukany ciąg nazków jest dłuższy niż dane w buforze lub
								nie znaleziono szukanej sekwencji
								bufor został przepełniony i istnieje ryzyko błędnego odczytu
		  - tail2virtualTail przyjmującą wartości:
		  	    - > 0 wskazującą miejsce w buforze, w którym dana sekwencja jest zlokalizowana
		  	    -   0 gdy nie znaleziono szukanej sekwencji lub znajduje się ona w pozycji 0 bufora
		  	  	 -
*/
template<typename T, size_t Size> circularBufferSearchResult CircularBuffer<T, Size>::SearchItemTail2Range(const void *item, size_t items, size_t range, bool tailPosUpdate){
	circularBufferSearchResult returnVAL;
	returnVAL.isFound =false;
	returnVAL.tail2virtualTail_ =0;
	if (!isOverflowed())
	{
		size_t searchingPosition = tail_;

		if (range > max_size_)
			range = max_size_;
		size_t toLinearedHead_ = tail_ + range;

		size_t itemsOnTheEnd;						//liczba pamięta ile danych memcmp powinno sprawdzić na końcu bufora kołowego, a ile na jego początku
		size_t tail2virtualTail_Distance = 0;

		char* cbuf_ = (char *) buf_;
		char* citem = (char *) item;

		while(searchingPosition+items <= toLinearedHead_)
		{
			searchingPosition = (searchingPosition) % max_size_;

			itemsOnTheEnd = max_size_- searchingPosition;			//sprawdzanie czy części danych nie trzeba szukać na początku bufora kołowego
			if (itemsOnTheEnd>=items)
				itemsOnTheEnd=items;

			if (memcmp((cbuf_+searchingPosition ), citem, sizeof (T)*itemsOnTheEnd)==0)				//sprawdzanie danych na końcu bufora kołowego
			{

				if (memcmp((cbuf_), citem+itemsOnTheEnd, sizeof (T)*(items-itemsOnTheEnd))==0)		//sprawdzanie danych na początku bufora kołowego
				{
					returnVAL.isFound = true;
					returnVAL.tail2virtualTail_ =tail2virtualTail_Distance/sizeof(T);
					if (tailPosUpdate==true)
					{
						tail_=searchingPosition;		//ustawnienie pozycji odczytu na miejsce taila
					}
					return returnVAL;					//znaleziono poprawną sekwencję
				}
			}
			searchingPosition++;
			if (searchingPosition == tail_)		//aby wyjśc z pętli nieskończonej jesli w buforze nie ma szukanego ciągu
				break;
			tail2virtualTail_Distance++;
		};
	}
	return returnVAL;		//szukany ciąg nazków jest dłuższy niż dane w buforze lub
						//nie znaleziono szukanej sekwencji
						//bufor został przepełniony i istnieje ryzyko błędnego odczytu
}


/**
* @brief  Sprawdza czy bufor kołowy zawiera interesujące nas dane.
* @note   Wyszukuje w zakresie od tail_ do head_
*
* @param  *item 		- wskaźnik do bufora zawierającego szukane informacje
* 		  items 		- ilość danych typu T jaka ma być wyszukana
* 		  range 		- zakres przeszukiwania
* 		  tailPosUpdate - informuje obiekt czy po znalezieniu szykanej wartości ustawić na nią tail_
* 		  					TRUE - wartośc domyśla oznacza, że tail_ powinien być uaktualniony
* 		  					FALSE - oznacza, że tail_ nie powinien być uaktualniony
* @note   n/a.
*
* @retval Zwraca strukturę circularBufferSearchResult o następujących polach:
		  - isFound przyjmujących wartości:
* 				- TRUE jeśli szukana wartośc została odnaleziona
* 				- FALSE jeśli: 	szukany ciąg nazków jest dłuższy niż dane w buforze lub
								nie znaleziono szukanej sekwencji
								bufor został przepełniony i istnieje ryzyko błędnego odczytu
		  - tail2virtualTail przyjmującą wartości:
				- > 0 wskazującą miejsce w buforze, w którym dana sekwencja jest zlokalizowana

*/
template<typename T, size_t Size> circularBufferSearchResult CircularBuffer<T, Size>::SearchItem(const void *item, size_t items, bool tailPosUpdate) {
	return SearchItemTail2Range (item, items, dataSize(), tailPosUpdate);
}


/**
* @brief  Sprawdza czy bufor kołowy zawiera interesujące nas dane.
* @note   Wyszukuje w zakresie od tail_ do head_
*
* @param  *item 		- wskaźnik do bufora zawierającego szukane informacje
* 		  items 		- ilość danych typu T jaka ma być wyszukana
* 		  tailPosUpdate - informuje obiekt czy po znalezieniu szykanej wartości ustawić na nią tail_
* 		  					TRUE - wartośc domyśla oznacza, że tail_ powinien być uaktualniony
* 		  					FALSE - oznacza, że tail_ nie powinien być uaktualniony
* @note   n/a.
*
* @retval Zwraca:
* 				- TRUE jeśli szukana wartośc została odnaleziona
* 				- FALSE jeśli: 	szukany ciąg nazków jest dłuższy niż dane w buforze lub
								nie znaleziono szukanej sekwencji
								bufor został przepełniony i istnieje ryzyko błędnego odczytu
*/
template<typename T, size_t Size> bool CircularBuffer<T, Size>::searchItem(const void *item, size_t items, bool tailPosUpdate) {
	return searchItemTail2Range(item, items, dataSize(), tailPosUpdate);
}




/**
* @brief  Sprawdza czy bufor kołowy zawiera interesujące nas dane.
* @note   Wyszukuje w zakresie od tail_ do tail_ + range
*
* @param  *item 		- wskaźnik do bufora zawierającego szukane informacje
* 		  items 		- ilość danych typu T jaka ma być wyszukana
* 		  range 		- zakres przeszukiwania
* 		  tailPosUpdate - informuje obiekt czy po znalezieniu szykanej wartości ustawić na nią tail_
* 		  					TRUE - wartośc domyśla oznacza, że tail_ powinien być uaktualniony
* 		  					FALSE - oznacza, że tail_ nie powinien być uaktualniony
* @note   n/a.
*
* @retval Zwraca:
* 				- TRUE jeśli szukana wartośc została odnaleziona
* 				- FALSE jeśli: 	szukany ciąg nazków jest dłuższy niż dane w buforze lub
								nie znaleziono szukanej sekwencji
								bufor został przepełniony i istnieje ryzyko błędnego odczytu
*/
template<typename T, size_t Size> bool CircularBuffer<T, Size>::searchItemTail2Range(const void *item, size_t items, size_t range, bool tailPosUpdate) {
	if (!isOverflowed())
	{
		size_t searchingPosition = tail_;
		if (range > max_size_)
			range = max_size_;
		size_t toLinearedHead_ = tail_ + range; //dataSize(head_, range);				//virtualny koniec bufora, gdyby był liniowym
		size_t itemsOnTheEnd;										//liczba pamięta ile danych memcmp powinno sprawdzić na końcu bufora kołowego, a ile na jego początku

		char* cbuf_ = (char *) buf_;								//potrzeba, aby przyjmować const void *item
		char* citem = (char *) item;								//potrzeba, aby przyjmować const void *item

		while(searchingPosition+items <= toLinearedHead_)
		{
			searchingPosition = (searchingPosition) % max_size_;

			itemsOnTheEnd = max_size_- searchingPosition;			//sprawdzanie czy części danych nie trzeba szukać na początku bufora kołowego
			if (itemsOnTheEnd>=items)
				itemsOnTheEnd=items;

			if (memcmp((cbuf_+searchingPosition ), citem, sizeof (T)*itemsOnTheEnd)==0)				//sprawdzanie danych na końcu bufora kołowego
			{
				if (memcmp((cbuf_), citem+itemsOnTheEnd, sizeof (T)*(items-itemsOnTheEnd))==0)		//sprawdzanie danych na początku bufora kołowego
				{
					if (tailPosUpdate==true)
						tail_=searchingPosition;		//ustawnienie pozycji odczytu na miejsce taila
					return true;					//znaleziono poprawną sekwencję
				}
			}
			searchingPosition++;
			if (searchingPosition == tail_)		//aby wyjśc z pętli nieskończonej jesli w buforze nie ma szukanego ciągu
				break;
		};
	}
	return false;		//szukany ciąg nazków jest dłuższy niż dane w buforze lub
						//nie znaleziono szukanej sekwencji
						//bufor został przepełniony i istnieje ryzyko błędnego odczytu
}

#endif
/* CIRCULARBUFFER_H_ */
