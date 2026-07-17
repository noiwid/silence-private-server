from helpers.constants import *
import json
import logging
import os

from pubsub import pub

log = logging.getLogger(LOGGER_NAME)

class MessageParser:

    def __init__(self):

        self.scooter_off = True
        self.off_statuses = [0,1,5]

        # load message parsing configuration
        with open(os.path.join(os.path.dirname(__file__), "Z_protocol_message_decode.json")) as message_configuration:
            self.message_decode = json.load(message_configuration)

        with open("scooter_status_definition.json") as status_definition:
            self.parameters = json.load(status_definition)

        with open(os.path.join(os.path.dirname(__file__), "RCAN_definition.json")) as RCAN_message_configuration:
            self.RCAN_message_configuration = json.load(RCAN_message_configuration)

        # Valid single-frame lengths derived from decode config — used to detect bundled frames.
        self._valid_z_lengths = {
            l
            for p in self.message_decode.values()
            for mt in p["message_type"]
            for l in mt["message_lenght"]
        }

        # Fields populated by extended CAN polling (_parse_extended_can).
        # These are reset to "None" when the scooter is off so consumers
        # don't see stale values (e.g. RPM=1341, driveMode=SPORT while
        # the scooter has been parked for hours).
        self._extended_can_keys = [
            "driveMode", "driveReady", "sidestandDown", "warningLights",
            "rangeByMode", "bmsFlags", "bmsCurrent",
            "batteryNTC1", "batteryNTC2", "batteryNTC3",
            "motorRPM", "motorPower", "busVoltage",
        ]

    def parse_message_from_scooter_protocol_Z(self, data):

        if len(data) > 0:
            log.debug(f"Parse received message protocol Z from scooter: {data}")

            # Handle bundled Z frames: when RCAN polling slows the comm loop,
            # multiple Z sub-frames get buffered and read as one big frame.
            # Format: Z[len_hi][len_lo][count][sub0][sub1]...[checksum]
            # Extract last sub-frame (most recent) and wrap in valid Z header.
            # Fix: check sub_count > 1 and not a known single-frame size,
            # instead of len > 200 (misses 182-byte dual-frame packets).
            if data[0] == 0x5A and len(data) >= 4 and len(data) not in self._valid_z_lengths:
                sub_count = data[3]
                if sub_count > 1:
                    sub_size = (len(data) - 4 - 2) // sub_count  # 4=header, 2=checksum
                    if sub_size > 0:
                        last_offset = 4 + (sub_count - 1) * sub_size
                        last_sub = data[last_offset : last_offset + sub_size]
                        # Rebuild a valid single-record Z frame:
                        # Z(1) + len(2) + count=1(1) + sub(88) + checksum(2) = 94
                        synth_len = sub_size + 4 + 2  # sub + header + checksum
                        data = bytes([0x5A]) + synth_len.to_bytes(2, 'big') + bytes([1]) + bytes(last_sub) + bytes(2)
                        log.debug(f"Extracted sub-frame {sub_count}/{sub_count}, synth len={len(data)}")

            try:
                for parameter in self.message_decode:
                    if self.message_decode[parameter]["disable_when_off"] and self.scooter_off:
                        self.parameters[parameter]["value"] = "None"
                    else:
                        for message_type in self.message_decode[parameter]["message_type"]:
                            try:
                                if message_type["message_first_char"] == int(data[0]) and len(data) in message_type["message_lenght"]:
                                    byte_start = message_type["message_byte_pos"][0]
                                    byte_end = message_type["message_byte_pos"][1]+1
                                    value = data[byte_start:byte_end]
                                    if self.message_decode[parameter]["data_type"] == "boolean":
                                        self.parameters[parameter]["value"] = int(value[0] & (1 << self.message_decode[parameter]["bit_pos"]) != 0)
                                    elif self.message_decode[parameter]["data_type"] == "numeric":
                                        self.parameters[parameter]["value"] = int.from_bytes(value, byteorder='big',signed=True) / self.message_decode[parameter]["divider"]
                                        if parameter == "status" and self.parameters[parameter]["value"] in self.off_statuses:
                                            self.scooter_off = True
                                        elif parameter == "status" and self.parameters[parameter]["value"] not in self.off_statuses:
                                            self.scooter_off = False
                                    elif self.message_decode[parameter]["data_type"] == "text":
                                        self.parameters[parameter]["value"] = str(value.decode())
                            except Exception:
                                log.exception(f"Exception in parsing parameter {parameter}")


                # Validate parsed data before publishing — the last Z frame before
                # shutdown often contains corrupted values (odo=867M, energy=-55923, etc.)
                odo_val = self.parameters.get("odo", {}).get("value")
                if odo_val is not None and odo_val != "None":
                    try:
                        if float(odo_val) > 1000000 or float(odo_val) < 0:
                            log.warning("Corrupt Z frame detected (odo=%s), skipping publish", odo_val)
                            return
                    except (ValueError, TypeError):
                        pass

                # When the scooter is off, the extended CAN fields (populated
                # only via $RCAN polling during movement) keep their stale
                # last-known values indefinitely. Reset them to "None" so
                # consumers see a clean state that matches reality (no RPM,
                # no drive mode, no bus voltage when the scooter is off).
                if self.scooter_off:
                    for key in self._extended_can_keys:
                        if key in self.parameters:
                            self.parameters[key]["value"] = "None"

                log.debug(f"Message protocol Z parsed: {self.parameters}")
                pub.sendMessage(TOPIC_SCOOTER_STATUS, scooter_status = self.parameters)

            except Exception:
                log.exception(f"Exception in handling message protocol Z {data}")

    def parse_message_from_scooter_protocol_astra(self, data):

        if len(data) > 0:
            log.debug(f"Parse received message protocol astra from scooter: {data}")
            try:
                data = data.decode()

                # $STMS snapshot frame (forced sync via the SYNC command).
                # Full status in one CSV line; decoded separately because the
                # field layout differs entirely from $RCAN.
                if data.startswith("$STMS,"):
                    self._parse_stms(data)
                    return

                self._parse_extended_can(data)

                for parameter in self.RCAN_message_configuration:
                    if data[:len(self.RCAN_message_configuration[parameter]["header"])] == self.RCAN_message_configuration[parameter]["header"]:
                        byte_pos = self.RCAN_message_configuration[parameter]["message_byte_pos"]
                        positions = data.split(",")
                        combined_HEX = positions[byte_pos[1]] + positions[byte_pos[0]]
                        self.parameters[parameter]["value"] = int(combined_HEX, 16)

                log.debug(f"Message protocol astra parsed: {self.parameters}")
                pub.sendMessage(TOPIC_SCOOTER_STATUS, scooter_status = self.parameters)

            except Exception:
                log.exception(f"Exception in handling message protocol astra {data}")

    def _parse_stms(self, data):
        """Decode a $STMS snapshot frame and publish scooter status.

        Field indices reverse-engineered then cross-checked against live
        Z-protocol telemetry from the same scooter:
          SOC=82, Vbat=55.6 (556/10), Tmax=25, Tmin=24, range=109,
          charged=331.5425 (1193553/3600), regen=28.2425 (101673/3600),
          discharged=321.8067 (1158504/3600), VIN=UCYSxxxxxxxxxxxxx (17 chars).

        Example frame:
          $STMS,0,82,25,24,556,0,435036987,0,0,330,330,0,109,0,
                1193553,101673,1158504,22,<ICCID>,<phone>,<VIN>

        Indices 7 (large counter), 8-12 and 14 are unidentified and left
        untouched so they don't clobber good Z data — notably odo, which
        $STMS does not carry (Z odo=6345 km != field 7).
        """
        parts = data.strip().split(",")

        # (csv_index, parameter_key, divider)
        numeric_fields = [
            (1, "status", 1),
            (2, "batterySOC", 1),
            (3, "batteryTempMax", 1),
            (4, "batteryTempMin", 1),
            (5, "batteryVoltage", 10),
            (6, "batteryCurrent", 10),
            (13, "range", 1),
            (15, "chargedEnergy", 3600),
            (16, "RegeneratedEnergy", 3600),
            (17, "DischargedEnergy", 3600),
            (18, "ambientTemp", 1),
        ]

        parsed_any = False
        for index, key, divider in numeric_fields:
            if index >= len(parts):
                continue
            raw = parts[index].strip()
            if raw == "":
                continue
            try:
                self.parameters[key]["value"] = int(raw) / divider
                parsed_any = True
            except (ValueError, KeyError):
                log.debug("STMS: cannot parse %s (index %s) value %r", key, index, raw)

        # VIN is the last CSV field; sanity-check it looks like a Silence VIN
        # before trusting it (guards against a truncated/garbled frame).
        if len(parts) >= 2:
            vin = parts[-1].strip()
            if vin.startswith("UCYS") and len(vin) >= 10:
                try:
                    self.parameters["VIN"]["value"] = vin
                    parsed_any = True
                except KeyError:
                    pass

        if not parsed_any:
            log.warning("STMS frame had no decodable fields: %s", data)
            return

        log.debug(f"Message $STMS parsed: {self.parameters}")
        pub.sendMessage(TOPIC_SCOOTER_STATUS, scooter_status=self.parameters)

    def _parse_extended_can(self, data):
        """Parse extended CAN data from $RCAN responses."""
        if not data.startswith("$RCAN,"):
            return

        parts = data.strip().split(",")
        if len(parts) < 3:
            return

        rcan_id = parts[1]

        try:
            # 0x280 - ECU: Drive mode + switches
            if rcan_id == "280" and len(parts) >= 5:
                byte0 = int(parts[3], 16)
                byte1 = int(parts[4], 16)
                self.parameters["driveReady"]["value"] = int(byte0 & 0x01 != 0)
                self.parameters["sidestandDown"]["value"] = int(byte0 & 0x08 != 0)
                mode_bits = (byte0 >> 4) & 0x03
                mode_map = {0: "OFF", 1: "ECO", 2: "SPORT", 3: "CITY"}
                self.parameters["driveMode"]["value"] = mode_map.get(mode_bits, "UNKNOWN")
                self.parameters["warningLights"]["value"] = int(byte1 & 0x01 != 0)

            # 0x300 - Range by current drive mode
            elif rcan_id == "300" and len(parts) >= 5:
                self.parameters["rangeByMode"]["value"] = int(parts[4], 16)

            # 0x182 - BMS flags
            elif rcan_id == "182" and len(parts) >= 4:
                self.parameters["bmsFlags"]["value"] = int(parts[3], 16)

            # parts layout: $RCAN,{ID},{len},{b0},{b1},{b2},{b3},{b4},{b5},{b6},{b7},OK
            #                  0     1    2    3    4    5    6    7    8    9    10   11

            # 0x181 - BMS live current (bytes 6-7, signed LE, /10 = amps)
            elif rcan_id == "181" and len(parts) >= 11:
                b6 = int(parts[9], 16)
                b7 = int(parts[10], 16)
                current_raw = b6 | (b7 << 8)
                if current_raw > 32767:
                    current_raw -= 65536
                self.parameters["bmsCurrent"]["value"] = round(current_raw / 10.0, 1)

            # 0x189 - Battery NTC temperatures (3 probes, bytes 2-7, /100 = celsius)
            elif rcan_id == "189" and len(parts) >= 11:
                ntc1 = int(parts[5], 16) | (int(parts[6], 16) << 8)
                ntc2 = int(parts[7], 16) | (int(parts[8], 16) << 8)
                ntc3 = int(parts[9], 16) | (int(parts[10], 16) << 8)
                self.parameters["batteryNTC1"]["value"] = round(ntc1 / 100.0, 1)
                self.parameters["batteryNTC2"]["value"] = round(ntc2 / 100.0, 1)
                self.parameters["batteryNTC3"]["value"] = round(ntc3 / 100.0, 1)

            # 0x391 - Motor RPM (bytes 4-5, unsigned LE)
            elif rcan_id == "391" and len(parts) >= 9:
                b4 = int(parts[7], 16)
                b5 = int(parts[8], 16)
                self.parameters["motorRPM"]["value"] = b4 | (b5 << 8)

            # 0x381 - Motor power/torque (bytes 2-3, signed LE)
            elif rcan_id == "381" and len(parts) >= 7:
                b2 = int(parts[5], 16)
                b3 = int(parts[6], 16)
                power_raw = b2 | (b3 << 8)
                if power_raw > 32767:
                    power_raw -= 65536
                self.parameters["motorPower"]["value"] = power_raw

            # 0x371 - Votol bus voltage (bytes 6-7, unsigned LE, /10 = volts)
            elif rcan_id == "371" and len(parts) >= 11:
                b6 = int(parts[9], 16)
                b7 = int(parts[10], 16)
                self.parameters["busVoltage"]["value"] = round((b6 | (b7 << 8)) / 10.0, 1)

        except (ValueError, IndexError, KeyError) as e:
            log.debug("Error parsing extended CAN %s: %s", rcan_id, e)

    def get_scooter_off_status(self):
        return self.scooter_off
