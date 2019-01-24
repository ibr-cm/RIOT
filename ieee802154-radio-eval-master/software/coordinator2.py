from time import sleep, time
import re
from clients.client import Client
from common.logging import Logger
from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy import create_engine
from threading import Lock

db_lock = Lock()

# Middle: serial-A502C1P4
#NODES = ['serial-A502C1OZ', 'serial-A502C1PH', 'serial-A502C1PV', 'serial-A502C1PE', 'serial-A502C1Q1', 'serial-A502C1QH', 'serial-A502C1P5', 'serial-A502C1PI', 'serial-A502C1P4']
# node ids: 7, ..., 16
NODES = ['serial-A502C1PO', 'serial-A502C1PV', 'serial-A502C1P4', 'serial-A502C1QH', 'serial-A502C1OZ', 'serial-A502C1PE', 'serial-A502C1PI', 'serial-A502C1P5', 'serial-A502C1PH', 'serial-A502C1Q1']
CHANNELS = [11, 15, 26]
ANGELS = [0,25,50,75,100,125,150,175]
POWERS = [-17, -7, 3]
PAYLOADS = ['short', 'loooooooooooooooooooooooooooooooooooooooong']

if False:
    CHANNELS = [11]
    POWERS = [-17]
    PAYLOADS = ['short']

logger = Logger(__file__)
Base = declarative_base()

class Round(Base):
    __tablename__ = 'round'
    id = Column(Integer, nullable=False, primary_key=True, autoincrement=True)
    channel = Column(Integer, nullable=False)
    power = Column(Integer, nullable=False)
    angle = Column(Integer, nullable=False)
    payload = Column(String(250), nullable=False)
    ts = Column(Float, nullable=False)

class TX(Base):
    __tablename__ = 'tx'
    serial_id = Column(String(250), nullable=False, primary_key=True)
    ts = Column(Float, nullable=False, primary_key=True)
    seq_nr = Column(Integer, nullable=False)
    round_id = Column(Integer)

class RX(Base):
    __tablename__ = 'rx'
    serial_id = Column(String(250), nullable=False, primary_key=True)
    ts = Column(Float, nullable=False, primary_key=True)
    rssi = Column(Integer, nullable=False)
    lqi = Column(Integer, nullable=False)
    node_id = Column(Integer, nullable=False)
    seq_nr = Column(Integer, nullable=False)
    round_id = Column(Integer)

class Corrupt(Base):
    __tablename__ = 'corrupt'
    serial_id = Column(String(250), nullable=False, primary_key=True)
    ts = Column(Float, nullable=False, primary_key=True)
    rssi = Column(Integer, nullable=False)
    lqi = Column(Integer, nullable=False)
    payload = Column(String(255), nullable=False)
    round_id = Column(Integer)

engine = create_engine('sqlite:////data/row_2.db')
Session = sessionmaker(bind=engine)
Base.metadata.create_all(engine)

re_tx = re.compile("^>(?P<seq_nr>[0-9]+)=(?P<checksum>[A-Fa-f0-9]+)$")
re_rx = re.compile("^<(?P<node_id>[0-9]+)\|(?P<seq_nr>[0-9]+)\|(?P<rssi>\-?[0-9]+)\|(?P<lqi>[0-9]+)=(?P<checksum>[A-Fa-f0-9]+)$")
# <-94|52|0|=A5
re_rx_corrupt = re.compile("^<(?P<rssi>-?[0-9]+)\|(?P<lqi>[0-9]+)\|(?P<length>[0-9]+)\|(?P<payload>(?: [A-Fa-f0-9]+)*)=(?P<checksum>[A-Fa-f0-9]+)$")

re_ifconfig = re.compile("^success: set (.*?) o(n|f) interface [0-9]+ to (.*?)$")

eval_round = None

def calculate_checksum(*args):
    s = 0
    for a in args:
        s += sum(bytes(a, 'utf-8'))
    return hex( s&0xFF )[2:].upper().zfill(2)

def handle_match_tx( src_id, ts, match_tx ):
    seq_nr = match_tx.group("seq_nr")
    checksum = match_tx.group("checksum")
    
    expected_checksum = calculate_checksum( seq_nr )
    if expected_checksum == checksum:
        db_lock.acquire()
        session = Session()
        session.add(TX( round_id=eval_round, serial_id = src_id, ts=ts, seq_nr = seq_nr ))
        session.commit()
        session.close()
        db_lock.release()
        #logger.log("[{}] TX: #{}".format( src_id, seq_nr ))
        return True
    else:
        logger.debug("Corrupt Line")
        return False

def handle_match_rx( src_id, ts, match_rx ):
    node_id = match_rx.group("node_id")
    seq_nr = match_rx.group("seq_nr")
    rssi = match_rx.group('rssi')
    lqi = match_rx.group('lqi')
    checksum = match_rx.group("checksum")
    expected_checksum = calculate_checksum( node_id, '|', seq_nr, '|', rssi, '|', lqi )

    if expected_checksum == checksum:
        db_lock.acquire()
        session = Session()
        session.add(RX(round_id=eval_round, serial_id = src_id, ts=ts, seq_nr = seq_nr, node_id = node_id, rssi = rssi, lqi = lqi))
        session.commit()
        session.close()
        db_lock.release()
        # TODO(rh): Add sqlite3.OperationalError
        # TODO(rh): Add sqlalchemy.exc.OperationalError / sqlite3.OperationalError
        #logger.log("[{}] RX: #{} from {}".format( src_id, node_id, seq_nr ))
        return True
    else:
        logger.debug("Corrupt Line {} != {}".format( expected_checksum, checksum) )
        return False

def handle_match_rx_corrupt( src_id, ts, match_rx_corrupt ):
    rssi = match_rx_corrupt.group('rssi')
    lqi = match_rx_corrupt.group('lqi')
    payload = match_rx_corrupt.group('payload').strip()
    logger.debug( "Corrupt: {}dBm, {}, {}".format( rssi, lqi, payload ) )
    db_lock.acquire()
    session = Session()
    session.add(Corrupt(round_id=eval_round, serial_id = src_id, ts=ts, rssi = rssi, lqi = lqi, payload = payload))
    session.commit()
    session.close()
    db_lock.release()
    return True

def handle_serial_rx(d):
    global eval_round
    src_id = d['src']['id'][0]
    ts = d['ts']
    line = d['line'].strip()

    if eval_round is None:
        return False

    match_ifconfig = re_ifconfig.match(line)
    if match_ifconfig is not None:
        return True
    match_tx = re_tx.match(line)
    if match_tx is not None:
        return handle_match_tx( src_id, ts, match_tx )
    match_rx = re_rx.match(line)
    if match_rx is not None:
        return handle_match_rx( src_id, ts, match_rx )
    match_rx_corrupt = re_rx_corrupt.match(line)
    if match_rx_corrupt is not None:
        return handle_match_rx_corrupt( src_id, ts, match_rx_corrupt  )
    return False

class PRREvalClient(Client):
    def __init__(self, args = None):
        super(PRREvalClient, self).__init__(args, logging = True)
        self.session = Session()

    def eval(self):
        global eval_round
#        for angle in [0,25,50,75,100,125,150,175]:
#            self.set_angle(angle)
#            sleep(0.5)
        for channel in CHANNELS:
            logger.log( "CHANNEL={} ".format(channel) )
            self.set_channel( channel  )
            sleep(0.5)
            for power in POWERS:
                logger.log( "POWER={} ".format(power) )
                self.set_txpower( power  )
                sleep( 0.5 )
                for payload in PAYLOADS:
                    logger.log( "PAYLOAD={} ".format(payload) )
                    self.send_all( "payload {}\n".format( payload )  )
                    sleep( 0.5 )
                    db_lock.acquire()
                    r = Round( angle=-1, channel=channel, power=power, payload=payload, ts=time() )
                    self.session.add( r )
                    self.session.commit()
                    eval_round = r.id
                    db_lock.release()
                    for sender in NODES:
                        self.send({'dst': {'id': sender}, 'type': 'serial_tx', 'line': 'tx 5 200000\n'})
                        # 5 packets, 200ms -> 1 second
                        sleep( 2 )
                        self.send({'dst': {'id': sender}, 'type': 'serial_tx', 'line': 'ifconfig 3 set state idle\n'})
                    # Delay after all sends
                    time.sleep( 5.0 )

    def run(self):
        while True:
            try:
                self.eval()
            except KeyboardInterrupt:
                logger.success("Stopped")
                break

    def set_angle(self, angle):
        logger.log("ANGLE={}".format( angle ))
        # {'module': ['SerialStepperMotorModule']}
        self.send({'dst': {'id': ['lctc-01-serialsteppermotormodule']}, 'type': 'set_position', 'position': angle})

    def set_channel(self, channel):
        #self.send_all("ifconfig 3 set state idle\n")
        self.send_all("ifconfig 3 set channel {}\n".format( channel ))

    def set_txpower(self, power):
        self.send_all("ifconfig 3 set power {}\n".format( power ))

    def send_all(self, line):
        self.send({'dst': {'id': NODES}, 'type': 'serial_tx', 'line': line})

    def callback_receive(self, data_dict):
        if 'type' in data_dict:
            if data_dict['type'] == 'serial_rx':
                if handle_serial_rx( data_dict ):
                    return True
                else:
                    logger.debug("[{}] {}".format( data_dict['src']['id'][0], data_dict['line'] ) )
                    return True
          
        logger.debug( data_dict )

if __name__ == '__main__':
    client = PRREvalClient()
    client.run()
