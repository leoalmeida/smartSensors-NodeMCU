#ifndef Collection_h
#define Collection_h

#include <Arduino.h>

typedef struct Profile {    
   String id;
   boolean publish;
   boolean subscribe;
   long sync;
   int access;
} profile_t;

template<typename N>
struct Node {
   N* value;
   struct Node<N> *next;
   struct Node<N> *previous;
};

profile_t * create_profile(String id, boolean publish, boolean subscribe){
  profile_t * profile = (profile_t *)malloc(sizeof(profile_t));
  profile->id = id;
  profile->publish = publish;
  profile->subscribe = subscribe;
  profile->sync = 0;
  profile->access = 0;
  return profile;
}

template<typename T> 
class LinkedList
{
  public:
    LinkedList();
    LinkedList<T>(String, boolean, boolean);
    ~LinkedList<T>();
    void push_first(String, boolean, boolean);
    void push_back(String, boolean, boolean);
    boolean pull_back();
    void pull_first();
    long length();
    boolean empty();
    boolean find(String);
    boolean pull_item(String);

  private:
    long size = 0;
    Node<T>* first;
    Node<T>* last;
    Node<T>* create(T*);
};

template<typename T> 
LinkedList<T>::LinkedList(){
      size = 0;
      first = NULL;
      last = NULL;
}

template<typename T> 
LinkedList<T>::LinkedList(String id, boolean publish, boolean subscribe){
        first = create(create_profile(id, publish, subscribe));
        last = first;
        size++;
}

template<typename T> 
LinkedList<T>::~LinkedList(){
      Node<T>* temp;
      while(this->first != NULL){
          temp = this->first->next;
          this->first->previous = NULL;
          this->first->next = NULL;
          free(this->first);
          size--;
          this->first = temp;
      };
};

template<typename T> 
void LinkedList<T>::push_first(String id, boolean publish, boolean subscribe){
              Node<T>* temp;
              temp = this->first;
              profile_t * profile = create_profile(id, publish, subscribe);
              this->first = create(profile);
              this->first->next = temp;
              size++;
}

template<typename T> 
void LinkedList<T>::push_back(String id, boolean publish, boolean subscribe){
              Node<T>* temp;
              temp = this->last;
              profile_t * profile = create_profile(id, publish, subscribe);
              this->last = create(profile);
              
              temp->next = this->last;
              this->last->previous = temp;
              
              size++;
}
template<typename T> 
boolean LinkedList<T>::pull_back(){
              
              if(this->empty()) Serial.println("Empty!!");
              else {
                Node<T> *temp = last;
                this->last = this->last->previous;
                this->last->next = NULL;
                free(temp);
                size--;
              }
              return true;
}


template<typename T> 
void LinkedList<T>::pull_first(){
              if(this->empty()) Serial.println("Empty!!");
              else {
                Node<T> *temp = this->first;
                this->first = this->first->next;
                this->first->previous = NULL;
                free(temp);
                size--;
              }
}

template<typename T> 
long LinkedList<T>::length(){
    return this->size;
}
            
template<typename T> 
boolean LinkedList<T>::empty(){
              if(this->first == NULL){
                return true;
              }
              return false;
}
            
template<typename T> 
boolean LinkedList<T>::find(String value){
              if(this->first->value->id == value) return true;
              if(this->last->value->id == value) return true;
              Node<T> *temp = this->first->next;
              while(temp != NULL){
                if (temp->value->id == value) return true;
                temp = temp->next;
              }  
              return false;
}

template<typename T> 
boolean LinkedList<T>::pull_item(String value){
            Node<T> *next, *actual, *prev;
            if(this->first == NULL){
                  Serial.println("Empty List!!");
                  return false;
            } else if (this->first->value->id == value){
              this->pull_first();
              return true;
            } else {
                actual = this->first->next;
                while(actual != NULL){
                  if (actual->value->id == value){
                    actual->previous->next = actual->next;
                    actual->next->previous = actual->previous;
                    free(actual);
                    Serial.println("removido com sucesso");
                    size--;
                    return true;
                  }
                  actual = actual->next;
                }
                return false;
            }
};

template<typename T>
Node<T>* LinkedList<T>::create(T* value){
          Node<T> * node = NULL;
          node = (Node<T>*)malloc(sizeof(Node<T>));
         
          node->value = value;
          node->next = NULL;
          node->previous = NULL;
          return node;
}

#endif
