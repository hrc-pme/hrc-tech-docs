# Software

---

## 1. ETH over SSH

> 在某些情況下不方便或無法使用 router 時，可以使用 Ethernet (乙太網路) 實體連接兩台裝置進行通訊。  
> 即便沒有 Wi-Fi 也能穩定監控目標機器的運作狀態。

筆記中紀錄的使用場景為：  
``` Linux Host OS --- ETH --- Linux Robot (Stretch 3) ```

### 1-1. procedure

> 與 Wi-Fi SSH 流程幾乎相同，只差在要手動設定靜態 IP。

1. Connect two Linux devices via Ethernet.
2. Set static IP addresses.
3. Enable SSH.
4. Connect using VSCode's Remote SSH extension.

### 1-2. static IP setup

> 插入乙太網路後，透過 `nmcli` 設定靜態 IP。

1. 檢查連線狀態：`sudo nmcli connection` 。
    
    幾點注意：
    
    * nmcli = NetworkManager Command-Line Interface。
    * 每行列出項目為一網路連線的設定，UUID 是各規則的唯一識別碼。
    * 綠色表正在使用；白灰色則表示已儲存但目前沒被使用的規則。
    * ETH 連線時就會找一個存在的規則來用。  
    若曾未設過 ETH 規則，系統會自動生成一個，通常是 `Wired connection 1`。
    * 設定檔可手動命名，並可指定綁定特定 MAC address 使用。

    ??? info "example"

        ![](../../assets/tutorial/nmcli_connection.png)

2. 分別修改 local host 和 remote host 的規則，以設定雙方 IP 位址。  
   設定完後請 `ping` 彼此的 IP 測試連線。
    
    * local host (192.168.10.1)
        ```
        sudo nmcli connection modify "<name>" \
          ipv4.method manual \
          ipv4.addresses 192.168.10.1/24 \
          ipv4.gateway ""
        nmcli connection up
        ```

    * remote host (192.168.10.2)
        ```
        sudo nmcli connection modify "<name>" \
          ipv4.method manual \
          ipv4.addresses 192.168.10.2/24 \
          ipv4.gateway ""
        nmcli connection up
        ```

至此即設定完成，其餘 SSH 基本操作省略。  
以下是一些 `nmcli` 其他常見功能。

1. 修改規則名稱。  
  ```
  sudo nmcli connection modify "<old-name>" connection.id "<new-name>"
  ```

    ??? info "example"

    ![](../../assets/tutorial/nmcli_name.png)

2. 顯示規則。  
  ```
  sudo nmcli connection show "<name>"
  ```

3. 切換規則。
   ```
   nmcli connection up <name> iface <eth_interface>
   ```

    ??? info "example"

        ```
        nmcli connection up stretch3 iface eth0
        ```
