#! /usr/bin/env python

import rospy
import grakn.client as graknclient
from graknrosservice.srv import Graknquery, GraknqueryResponse
from std_msgs.msg import String


class GraknClient:
    # To communicate with grakn KB
    def __init__(self, keyspace="ah_shelf", uri="localhost:48555"):
        self._keyspace = keyspace
        self._client = graknclient.GraknClient(uri)
        self._session = self._client.session(keyspace=keyspace)

    def close(self):
        self._session.close()

    def readKB(self):
        return self._session.transaction().read()

    def writeKB(self):
        return self._session.transaction().write()

    def query(self, query, querytype="read", infer=False):
        if querytype == "read":
            read = self.readKB()
            reply = read.query(query, infer)
            return reply
        else:
            write = self.writeKB()
            write.query(query)
            write.commit()
            return True

    def update_query(self, deletequery, updateequery):
        write = self.writeKB()
        write.query(deletequery)
        write.commit()
        write = self.writeKB()
        write.query(updateequery)
        write.commit()
        return True


class GraknROSServiceClass(object):
    # Ros clientserver to process the request made
    def __init__(self, name):
        self._action_name = name
        self._s = rospy.Service(self._action_name, Graknquery, self.execute_cb)
        self.gc = GraknClient(keyspace="ah_shelf")
        worldinitialisation = [' insert',
                               ' $Shelf isa Shelf, has shelf_id 1, has total_capacity 4,',
                               ' has width 2, has depth 2;',
                               ' $Product-type isa Product-type, has product_type "milk";',
                               ' $man1 isa Manipulability, has manipulability false;',
                               ' $man2 isa Manipulability, has manipulability true;']
        worldinitialisation = "".join(worldinitialisation)
        if self.gc.query(worldinitialisation, querytype="write", infer=True):
            rospy.loginfo("World state Initialised...")
        rospy.loginfo("Action server started...")

    def execute_cb(self, request):
        response = "Failed"
        hiddenstate = False

        if request.query_type == "observeshelf":
            observeshelf = [' match',
                            ' $Shelf isa Shelf, has shelf_id 1;',
                            ' $Product-type isa Product-type, has product_type "milk";',
                            ' insert',
                            ' $position1 isa Position, has pos_x 1, has pos_y 2;',
                            ' $position2 isa Position, has pos_x 2, has pos_y 1;',
                            ' $Pose1(position_: $position1) isa POSE_relation;',
                            ' $Pose2(position_: $position2) isa POSE_relation;',
                            ' $OBJECT1(pose_: $Pose1, shelf_: $Shelf, product_:',
                            ' $Product-type) isa OBJECT_relation,',
                            ' has action_execution "noaction", ',
                            ' has detectionmethod "sight"; ',
                            ' $OBJECT2(pose_: $Pose2, shelf_: $Shelf, product_:',
                            ' $Product-type) isa OBJECT_relation,',
                            ' has action_execution "noaction",',
                            ' has detectionmethod "sight";']
            observeshelf = "".join(observeshelf)
            if self.gc.query(observeshelf, querytype="write", infer=True):
                response = ['Objects observed at location (1,0) and (0,1) on',
                            'sheld with id 1. World state updated with observed objects']
                response = "".join(response)

        elif request.query_type == "executeaction":
            executeaction = [' match $obj(pose_:$pose,shelf_:$shelf,product_:$product_) isa OBJECT_relation, has action_execution $exe;',
                             ' $pose(position_:$position) isa POSE_relation;',
                             ' $position has pos_y 1;',
                             ' delete $obj has action_execution $exe;']
            executeaction = "".join(executeaction)
            self.gc.query(executeaction, querytype="write", infer=False)

            executeaction = [' match $obj(pose_:$pose,shelf_:$shelf,product_:$product_) isa OBJECT_relation;',
                             ' $pose(position_:$position) isa POSE_relation;',
                             ' $position has pos_y 1;',
                             ' insert $obj has action_execution "unsuccessful";']
            executeaction = "".join(executeaction)
            if self.gc.query(executeaction, querytype="write", infer=False):
                response = ['Pushing action on object at location (1,0) has failed. ',
                            'Hidden states present in the model.']
                response = "".join(response)

        elif request.query_type == "searchhiddenstates":
            currentstate = [' match $obj(pose_:$pose,shelf_:$shelf,product_:$product) isa OBJECT_relation, has action_execution $exe;',
                            ' $position has pos_y $pos_y, has pos_x $pos_x;',
                            ' $pose(position_:$position) isa POSE_relation;',
                            ' $shelf has depth $depth;',
                            ' $pos_y < $depth;',
                            ' $exe == "unsuccessful";',
                            ' get $pos_y,$pos_x,$shelf,$product;']
            currentstate = "".join(currentstate)

            for _, result in enumerate(self.gc.query(currentstate, infer=True)):
                pos_y = result.map().get('pos_y').value()
                pos_x = result.map().get('pos_x').value()
                shelf = result.map().get('shelf').id
                product = result.map().get('product').id
                hiddenstate = True

            if hiddenstate:
                updatedstate = ['match $shelf isa Shelf; $shelf id ' + str(shelf) + ';',
                                ' $product isa Product-type; $product id ' +
                                str(product) + ';',
                                ' insert (pose_:$pose,shelf_:$shelf ,product_: $product) isa OBJECT_relation,',
                                ' has detectionmethod "inference",',
                                ' has action_execution "noaction";',
                                ' $pose(position_:$position) isa POSE_relation;',
                                ' $position isa Position, has pos_y ' +
                                str(pos_y+1) + ', has pos_x ' + str(pos_x) + ';']
                updatedstate = "".join(updatedstate)
                if self.gc.query(updatedstate, querytype="write", infer=True):
                    response = 'Hidden state was present at(' + \
                        str(pos_x)+','+str(pos_y+1) + \
                        ') position updated. Updated world model with hidden states.'
                    response = "".join(response)
            else:
                response = "No hidden state. All objects are visible"

        rospy.loginfo(response)
        return GraknqueryResponse(response)


if __name__ == '__main__':
    rospy.init_node('grakn_ros')
    server = GraknROSServiceClass(rospy.get_name())
    rospy.spin()
