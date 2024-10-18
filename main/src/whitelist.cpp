//
// Created by Kurosu Chan on 2023/10/10.
//

#include "whitelist.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <functional>

#ifdef ESP32
#define LOG_ERR(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#elif defined(SIMPLE_LOG)
#define LOG_ERR(tag, fmt, ...) LOG_E(tag, fmt, ##__VA_ARGS__)
#else
#define LOG_ERR(tag, fmt, ...) // Define an empty macro if none of the conditions are met
#endif

#ifdef ESP32
#define LOG_INFO(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#elif defined(SIMPLE_LOG)
#define LOG_INFO(tag, fmt, ...) LOG_I(tag, fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(tag, fmt, ...) // Define an empty macro if none of the conditions are met
#endif

namespace white_list {
// decode one
using addr_fn = std::function<bool(Addr)>;
using name_fn = std::function<bool(Name)>;

// well. field item is a union, so you just only have one field set
void set_decode_white_item_addr(::WhiteItem &item, const addr_fn &write_addr) {
  item.item.mac.funcs.decode = [](pb_istream_t *stream, const pb_field_iter_t *field, void **arg) {
    if (arg == nullptr) {
      return false;
    }
    auto addr = Addr{};
    // Use pb_read instead, length of the string is available in stream->bytes_left.
    // https://jpa.kapsi.fi/nanopb/docs/concepts.html#decoding-callbacks
    // https://github.com/nanopb/nanopb/blob/09234696e0ef821432a8541b950e8866f0c61f8c/tests/callbacks/decode_callbacks.c#L10
    // https://en.cppreference.com/w/cpp/algorithm/swap
    // https://github.com/nanopb/nanopb/blob/master/tests/oneof_callback/decode_oneof.c
    if (stream->bytes_left < BLE_MAC_ADDR_SIZE) {
      LOG_ERR("white_list", "field length is not enough for bluetooth address. expected: %d, actual: %d", BLE_MAC_ADDR_SIZE, static_cast<int>(stream->bytes_left));
      return false;
    }

    const auto &w = *static_cast<addr_fn *>(*arg);
    if (!pb_read(stream, addr.addr.data(), BLE_MAC_ADDR_SIZE)) {
      LOG_ERR("white_list", "failed to read mac");
      return false;
    }
    return w(addr);
  };
  item.item.mac.arg = const_cast<addr_fn *>(&write_addr);
}

void set_decode_white_item_name(::WhiteItem &item, const name_fn &write_name) {
  item.item.name.funcs.decode = [](pb_istream_t *stream, const pb_field_iter_t *field, void **arg) {
    if (arg == nullptr) {
      return false;
    }
    auto name     = Name{};
    const auto &w = *static_cast<name_fn *>(*arg);
    // the 0x00 in the end?
    name.name.resize(stream->bytes_left + 1);
    if (!pb_read(stream, reinterpret_cast<pb_byte_t *>(name.name.data()), stream->bytes_left)) {
      LOG_ERR("white_list", "failed to read name");
      return false;
    }
    return w(std::move(name));
  };
  item.item.name.arg = const_cast<name_fn *>(&write_name);
}

bool set_encode_white_item(::WhiteItem &pb_item, const item_t &item) {
  if (std::holds_alternative<Name>(item)) {
    const auto &item_name = std::get<Name>(item);
    pb_item.which_item    = WhiteItem_name_tag;
    //  It can write as many or as few fields as it likes. For example,
    //  if you want to write out an array as repeated field, you should do it all in a single call.
    pb_item.item.name.funcs.encode = [](pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
      const auto &name = *static_cast<const Name *>(*arg);
      if (!pb_encode_tag_for_field(stream, field)) {
        return false;
      }
      return pb_encode_string(stream, reinterpret_cast<const uint8_t *>(name.name.data()), name.name.size());
    };
    pb_item.item.name.arg = const_cast<Name *>(&item_name);
    return true;
  } else if (std::holds_alternative<Addr>(item)) {
    const auto &item_mac          = std::get<Addr>(item);
    pb_item.which_item            = WhiteItem_mac_tag;
    pb_item.item.mac.funcs.encode = [](pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
      const auto &addr = *reinterpret_cast<const Addr *>(*arg);
      if (!pb_encode_tag_for_field(stream, field)) {
        return false;
      }
      return pb_encode_string(stream, addr.addr.data(), addr.addr.size());
    };
    pb_item.item.mac.arg = const_cast<Addr *>(&item_mac);
    return true;
  } else {
    return false;
  }
}

void set_encode_white_list(::WhiteList &pb_list, list_t &list) {
  pb_list.items.funcs.encode = [](pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
    const auto &list = *reinterpret_cast<const list_t *>(*arg);
    for (const auto &item : list) {
      // not very sure of creating a new item here
      ::WhiteItem pb_item;
      // Same as pb_encode_tag, except takes the parameters from a pb_field_iter_t structure.
      if (!set_encode_white_item(pb_item, item)) {
        LOG_ERR("white_list", "failed to set encode function.");
        return false;
      }
      if (!pb_encode_tag_for_field(stream, field)) {
        LOG_ERR("white_list", "failed to encode tag.");
        return false;
      }
      if (!pb_encode_submessage(stream, WhiteItem_fields, &pb_item)) {
        LOG_ERR("white_list", "failed to encode submessage");
        return false;
      }
    }
    return true;
  };
  pb_list.items.arg = &list;
}

bool marshal_white_list_response(pb_ostream_t *ostream, ::WhiteListResponse &pb_response, response_t &response) {
  if (std::holds_alternative<list_t>(response)) {
    pb_response.has_list = true;
    auto &l              = std::get<list_t>(response);
    set_encode_white_list(pb_response.list, l);
    return pb_encode(ostream, WhiteListResponse_fields, &pb_response);
  } else if (std::holds_alternative<::WhiteListErrorCode>(response)) {
    auto &e          = std::get<::WhiteListErrorCode>(response);
    pb_response.code = e;
    return pb_encode(ostream, WhiteListResponse_fields, &pb_response);
  } else {
    return false;
  }
}

/**
 * @brief Get the tag from istream without mutating it. Useful for oneof.
 * @param istream the stream to get tag from (usually is a parameter from a decode callback)
 */
uint32_t pb_get_tag(const pb_istream_t *istream) {
  pb_wire_type_t wire_type;
  uint32_t tag;
  // `pb_decode_tag` will mutate the original stream, create a copy
  pb_istream_t s_copy = *istream;
  bool eof;
  pb_decode_tag(&s_copy, &wire_type, &tag, &eof);
  return tag;
}

struct DecodeWhiteListCallbacks {
  name_fn name;
  addr_fn addr;
};

void set_decode_white_list(::WhiteList &list, DecodeWhiteListCallbacks &callbacks) {
  auto white_list_decode = [](pb_istream_t *stream, const pb_field_iter_t *field, void **arg) {
    const auto TAG = "unmarshal_set_white_list_callback";
    // https://stackoverflow.com/questions/73529672/decoding-oneof-nanopb
    auto &cbs = *static_cast<DecodeWhiteListCallbacks *>(*arg);
    if (field->tag == WhiteList_items_tag) {
      ::WhiteItem item = WhiteItem_init_zero;

      // https://github.com/nanopb/nanopb/blob/09234696e0ef821432a8541b950e8866f0c61f8c/examples/using_union_messages/decode.c#L24
      auto tag = pb_get_tag(stream);

      switch (tag) {
        case WhiteItem_name_tag:
          item.which_item = tag;
          set_decode_white_item_name(item, cbs.name);
          break;
        case WhiteItem_mac_tag:
          item.which_item = tag;
          set_decode_white_item_addr(item, cbs.addr);
          break;
        default:
          return false;
      }
      auto ok = pb_decode(stream, WhiteItem_fields, &item);
      if (!ok) {
        if (stream->errmsg != nullptr) {
          LOG_ERR(TAG, "decode error: %s", stream->errmsg);
        } else {
          LOG_ERR(TAG, "decode error");
        }
        return false;
      }
    }
    return true;
  };
  list.items.funcs.decode = white_list_decode;
  list.items.arg          = &callbacks;
}

etl::optional<request_t>
unmarshal_while_list_request(pb_istream_t *istream, ::WhiteListRequest &request) {
  auto tag = pb_get_tag(istream);
  switch (tag) {
    case WhiteListRequest_set_tag: {
      list_t result{};
      auto cbs = DecodeWhiteListCallbacks{
          .name = [&result](auto name) {
            result.emplace_back(item_t{name});
            return true; },
          .addr = [&result](auto addr) {
            result.emplace_back(item_t{addr});
            return true; },
      };
      set_decode_white_list(request.set, cbs);
      auto ok = pb_decode(istream, WhiteListRequest_fields, &request);
      if (!ok) {
        LOG_ERR("white_list", "failed to decode set");
        return etl::nullopt;
      }
      return request_t{std::move(result)};
    }
    case WhiteListRequest_command_tag: {
      auto ok      = pb_decode(istream, WhiteListRequest_fields, &request);
      auto command = request.command;
      if (!ok) {
        LOG_ERR("white_list", "failed to decode command");
        return etl::nullopt;
      }
      return request_t{command};
    }
    default:
      return etl::nullopt;
  }
}

bool marshal_white_list(pb_ostream_t *ostream, ::WhiteList &pb_list, list_t &list) {
  set_encode_white_list(pb_list, list);
  return pb_encode(ostream, WhiteList_fields, &pb_list);
}

etl::optional<list_t>
unmarshal_white_list(pb_istream_t *istream, ::WhiteList &pb_list) {
  list_t result;
  // https://github.com/nanopb/nanopb/blob/master/tests/oneof_callback/oneof.proto
  auto cbs = DecodeWhiteListCallbacks{
      .name = [&result](auto name) {
        result.emplace_back(item_t{name});
        return true; },
      .addr = [&result](auto addr) {
        result.emplace_back(item_t{addr});
        return true; },
  };
  set_decode_white_list(pb_list, cbs);
  auto ok = pb_decode(istream, WhiteList_fields, &pb_list);
  if (istream->errmsg != nullptr) {
    LOG_ERR("white_list", "stream->errmsg %s", istream->errmsg);
  }
  if (!ok) {
    LOG_ERR("white_list", "failed to decode response");
    return etl::nullopt;
  }
  return result;
}
}
