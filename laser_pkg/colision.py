#---------------------------- ----------- FUNCION PARA PUBLICAR COLISION ---------------------------------------#
def colision (self,a):
    msg = Int8()
    msg.data = a
    self.publisher_.publish(msg)
    print('sending colision alarm: ',a)